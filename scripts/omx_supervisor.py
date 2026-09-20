#!/usr/bin/env python3
"""Tkinter control window for lerobot-record / lerobot-teleoperate.

Moves the whole interactive surface of a recording session out of the terminal and
into a window: the dataset-name prompt, live episode status, the per-episode controls
(finish / re-record / end session) and a graceful Stop / Exit button.

Stop / Exit shuts the child down along an escalation ladder:

    1. SIGINT      -> LeRobot's cooperative stop (finish the frame, save, disable
                      torque, close the serial port, shut Rerun down)
    2. SIGINT #2   -> KeyboardInterrupt, still unwinding the normal finally: path
    3. SIGTERM     -> conventional termination request
    4. SIGKILL     -> last resort, whole process tree

SIGKILL is never the first step: it gives LeRobot no chance to disable torque, and a
follower joint left energised under load latches a servo hardware error that only a
power cycle clears.

How the buttons reach LeRobot
-----------------------------
LeRobot's recording controls come from ``TerminalKeyListener``, which reads the
controlling TTY (the only backend available on Wayland, where pynput cannot capture).
So the child is given a **pseudo-terminal** as its stdin/stdout/stderr: ``isatty()``
stays true, the listener works unchanged, and a button press is just a byte written
into the pty master. No LeRobot code is involved.

Everything the child writes is pumped back to the real terminal verbatim, so the
familiar log — cursor-repositioning live tables included — is unchanged, and keys
typed in the terminal are still forwarded to the child.
"""

from __future__ import annotations

import argparse
import collections
import errno
import fcntl
import os
import pty
import queue
import re
import signal
import struct
import subprocess
import sys
import termios
import threading
import time
import tty

try:
    import psutil
except ImportError:  # pragma: no cover - psutil ships with the omx env
    psutil = None


# --- shutdown ladder timings (seconds) -------------------------------------------
# Stage 1 is generous on purpose: a graceful stop still has to save the current
# episode and flush the video encoder, which can take a while on a long episode.
GRACE_COOPERATIVE = 120.0
GRACE_INTERRUPT = 30.0
GRACE_TERM = 15.0

POLL_INTERVAL = 0.25

# Dataset names become both a directory and a repo id, so keep them boring.
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
ERROR_RE = re.compile(r"(?:[A-Za-z_.]*(?:Error|Exception)\b.*|^ERROR\b.*)")

# Banners printed by lerobot-record, mapped to a status line and a session state.
# The state drives which buttons are enabled and what they are called, so the window
# always reflects what the recorder is actually doing rather than what we last sent.
EVENT_PATTERNS: list[tuple[re.Pattern, str | None, str | None]] = [
    (re.compile(r"RECORDING EPISODE (\d+)(?: / (\d+))?"), None, None),
    (re.compile(r"ARMED - PRESS START"), "Armed — press Start to begin capture", "armed"),
    (re.compile(r"RECORDING IS ACTIVE NOW"), "Recording", "recording"),
    (re.compile(r"RECORDING RESUMED"), "Recording", "recording"),
    (re.compile(r"RECORDING PAUSED"), "Paused — capture suspended, arm still live", "paused"),
    (re.compile(r"RECORDING STOPPED"), "Episode finished", "busy"),
    (re.compile(r"RESET TIME"), "Reset the scene, then press Finish Reset", "reset"),
    (re.compile(r"RESET FINISHED"), "Reset finished", "busy"),
    (re.compile(r"DISCARDING EPISODE (\d+)"), "Discarded episode {0} — will re-record", "busy"),
    (re.compile(r"SAVING EPISODE (\d+)"), "Saving episode {0} — please wait", "saving"),
    (re.compile(r"EPISODE (\d+) SAVED"), "Episode {0} saved", "busy"),
    (re.compile(r"MOVING FOLLOWER TO LEADER POSE"), "Moving follower to leader pose…", "busy"),
    (re.compile(r"GRACEFUL STOP REQUESTED"), "Stopping — LeRobot is cleaning up…", "stopping"),
]
EPISODE_RE = re.compile(r"RECORDING EPISODE (\d+)(?: / (\d+))?")
PROGRESS_RE = re.compile(r"Completed:\s*(\d+)(?:\s*/\s*(\d+))?")
APPROACH_RE = re.compile(r"\[approach\]\s*(.+)")


class Pty:
    """A pseudo-terminal pair for the child, sized to match the real terminal."""

    def __init__(self):
        self.master, self.slave = pty.openpty()

        # Mirror the real terminal's size so anything that queries it lays out right.
        try:
            size = fcntl.ioctl(sys.stdout.fileno(), termios.TIOCGWINSZ, b"\0" * 8)
            fcntl.ioctl(self.slave, termios.TIOCSWINSZ, size)
        except (OSError, ValueError):
            pass

        # Echo off from the start: the same pty carries the child's output, so keys we
        # inject must not be echoed back into the log before LeRobot's listener starts.
        try:
            attrs = termios.tcgetattr(self.slave)
            attrs[3] &= ~termios.ECHO  # lflags
            termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        except termios.error:
            pass

    def close_slave(self) -> None:
        """Drop our copy of the slave so the master reports EOF when the child exits."""
        if self.slave is not None:
            os.close(self.slave)
            self.slave = None

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except OSError:
            pass

    def close(self) -> None:
        for attr in ("slave", "master"):
            fd = getattr(self, attr, None)
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
                setattr(self, attr, None)


class ChildProcess:
    """The supervised LeRobot process plus everything it spawned."""

    def __init__(self, cmd: list[str], term: Pty):
        self.cmd = cmd
        self.term = term
        self.proc: subprocess.Popen | None = None
        self._descendants: set[int] = set()
        self._lock = threading.Lock()

    def start(self) -> None:
        env = dict(os.environ)
        # The child talks to a pty, so Python would line-buffer anyway; make it explicit
        # so status banners reach the window the moment they are printed.
        env["PYTHONUNBUFFERED"] = "1"
        # No start_new_session: staying in the terminal's process group keeps signal
        # handling and job control behaving the way the launcher script expects.
        self.proc = subprocess.Popen(
            self.cmd,
            stdin=self.term.slave,
            stdout=self.term.slave,
            stderr=self.term.slave,
            env=env,
            close_fds=True,
        )
        self.term.close_slave()

    @property
    def pid(self) -> int:
        assert self.proc is not None
        return self.proc.pid

    def is_alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    @property
    def returncode(self) -> int | None:
        return None if self.proc is None else self.proc.returncode

    def track_descendants(self) -> None:
        """Remember the child's descendants (Rerun viewer, camera/encoder workers).

        Recorded while they are still attached, so survivors can be swept afterwards
        even though an orphaned Rerun viewer gets reparented to init.
        """
        if psutil is None or self.proc is None:
            return
        try:
            parent = psutil.Process(self.proc.pid)
            found = {c.pid for c in parent.children(recursive=True)}
        except (psutil.Error, OSError):
            return
        with self._lock:
            self._descendants |= found

    def signal(self, sig: signal.Signals) -> bool:
        """Signal the child only — not the process group, which contains us too."""
        if not self.is_alive():
            return False
        try:
            os.kill(self.pid, sig)
            return True
        except OSError:
            return False

    def wait(self, timeout: float, on_tick=None) -> bool:
        """Wait up to *timeout* for exit. Returns True if the child exited."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.is_alive():
                return True
            self.track_descendants()
            if on_tick is not None:
                on_tick(max(0.0, deadline - time.monotonic()))
            time.sleep(POLL_INTERVAL)
        return not self.is_alive()

    def reap(self) -> None:
        if self.proc is not None:
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass

    def kill_tree(self, log) -> None:
        """SIGKILL the child and everything it left behind."""
        if self.is_alive():
            log("Stage 4: SIGKILL (forced).")
            try:
                os.kill(self.pid, signal.SIGKILL)
            except OSError:
                pass
            self.reap()

    def sweep_survivors(self, log) -> None:
        """Terminate anything the child spawned and left running (Rerun, cameras)."""
        if psutil is None:
            return
        self.track_descendants()
        with self._lock:
            pids = set(self._descendants)
        pids.discard(os.getpid())

        survivors = []
        for pid in pids:
            try:
                p = psutil.Process(pid)
                if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                    survivors.append(p)
            except (psutil.Error, OSError):
                continue

        if not survivors:
            log("No orphaned child processes left behind.")
            return

        for p in survivors:
            try:
                log(f"Terminating leftover process {p.pid} ({p.name()}).")
                p.terminate()
            except (psutil.Error, OSError):
                pass

        gone, alive = psutil.wait_procs(survivors, timeout=5)
        for p in alive:
            try:
                log(f"Force killing unresponsive process {p.pid}.")
                p.kill()
            except (psutil.Error, OSError):
                pass
        psutil.wait_procs(alive, timeout=3)


class OutputPump:
    """Copy the child's pty output to the real terminal, parsing status on the way."""

    def __init__(self, term: Pty, emit):
        self.term = term
        self.emit = emit
        self._buf = b""
        self._thread: threading.Thread | None = None
        # Kept so a non-zero exit can be explained in the window instead of only in
        # terminal scrollback.
        self.recent: collections.deque[str] = collections.deque(maxlen=60)

    def last_error(self) -> str:
        for line in reversed(self.recent):
            if ERROR_RE.search(line):
                return line
        for line in reversed(self.recent):
            if line.strip():
                return line
        return "See the terminal for details."

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def join(self, timeout: float = 2.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _run(self) -> None:
        out = sys.stdout.buffer
        while True:
            try:
                data = os.read(self.term.master, 4096)
            except OSError as e:
                # EIO is how Linux reports "every slave is closed", i.e. the child is gone.
                if e.errno not in (errno.EIO, errno.EBADF):
                    continue
                break
            if not data:
                break

            out.write(data)
            out.flush()
            self._parse(data)

    def _parse(self, data: bytes) -> None:
        self._buf += data
        # Split on either newline style; the live tables use \r to repaint.
        parts = re.split(rb"[\r\n]", self._buf)
        self._buf = parts.pop()
        for raw in parts:
            try:
                line = ANSI_RE.sub("", raw.decode("utf-8", "replace")).strip()
            except Exception:
                continue
            if line:
                self.recent.append(line)
                self._dispatch(line)

    def _dispatch(self, line: str) -> None:
        m = EPISODE_RE.search(line)
        if m:
            total = m.group(2)
            self.emit("episode", f"Episode {m.group(1)}" + (f" of {total}" if total else ""))
            return

        m = PROGRESS_RE.search(line)
        if m:
            total = m.group(2)
            done = m.group(1)
            self.emit("progress", f"Saved {done}" + (f" of {total}" if total else "") + " episode(s)")
            return

        for pattern, template, state in EVENT_PATTERNS:
            m = pattern.search(line)
            if m:
                if template:
                    self.emit("status", template.format(*m.groups()))
                if state:
                    self.emit("state", state)
                return

        m = APPROACH_RE.search(line)
        if m:
            self.emit("status", m.group(1))


class InputForwarder:
    """Forward keys typed in the real terminal into the child's pty."""

    def __init__(self, term: Pty):
        self.term = term
        self._fd = None
        self._old = None
        self._running = False

    def start(self) -> None:
        if not sys.stdin.isatty():
            return
        try:
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            attrs = termios.tcgetattr(self._fd)
            attrs[3] &= ~termios.ECHO
            termios.tcsetattr(self._fd, termios.TCSADRAIN, attrs)
        except (termios.error, ValueError, OSError):
            self._fd = self._old = None
            return

        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        import select

        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.2)
                if ready:
                    data = os.read(self._fd, 64)
                    if data:
                        self.term.write(data)
            except (OSError, ValueError):
                break

    def stop(self) -> None:
        self._running = False
        if self._fd is not None and self._old is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            except (termios.error, ValueError, OSError):
                pass
            self._old = None


class Supervisor:
    def __init__(self, args):
        self.args = args
        self.title = args.title
        self.term = Pty()
        self.child: ChildProcess | None = None
        self.pump: OutputPump | None = None
        self.forwarder = InputForwarder(self.term)
        self.messages: queue.Queue[tuple[str, object]] = queue.Queue()

        # Single-shot guard: repeated Stop clicks (or Stop + a terminal Ctrl+C)
        # must not restart or duplicate the ladder.
        self._stop_lock = threading.Lock()
        self._stopping = False
        self._finished = threading.Event()
        self._started = threading.Event()
        self.exit_code = 0
        self.dataset_name: str | None = None

        self.state = "starting"
        self._failed = False
        self._countdown_left: int | None = None
        self._countdown_job = None

        self.root = None
        self.status_var = None
        self.progress_var = None
        self.episode_var = None
        self.header_var = None
        self.stop_button = None
        self.toggle_button = None
        self.finish_button = None
        self.discard_button = None
        self.session_button = None
        self.control_buttons: list = []
        self.log_widget = None
        self.setup_frame = None
        self.running_frame = None
        self.name_entry = None
        self.name_error_var = None

    # --- logging ----------------------------------------------------------------
    def log(self, message: str) -> None:
        print(f"[stop-gui] {message}", flush=True)
        self.messages.put(("log", message))

    def gui_log(self, message: str) -> None:
        """Log to the window only — the line is already on the terminal."""
        self.messages.put(("log", message))

    def status(self, message: str) -> None:
        self.messages.put(("status", message))

    def emit(self, kind: str, payload: str) -> None:
        self.messages.put((kind, payload))

    # --- launching --------------------------------------------------------------
    def launch(self, dataset_name: str | None) -> None:
        cmd = list(self.args.command)
        if dataset_name is not None:
            cmd = [part.replace("{DATASET}", dataset_name) for part in cmd]
            self.dataset_name = dataset_name
            if self.args.dataset_name_out:
                try:
                    with open(self.args.dataset_name_out, "w") as fh:
                        fh.write(dataset_name)
                except OSError as e:
                    self.log(f"Could not record the dataset name: {e}")

        self.child = ChildProcess(cmd, self.term)
        self.child.start()
        self.pump = OutputPump(self.term, self.emit)
        self.pump.start()
        self.forwarder.start()
        self._started.set()
        self.log(f"Started PID {self.child.pid}: {' '.join(cmd)}")

    def send_key(self, key: str, label: str) -> None:
        if self.child is None or not self.child.is_alive() or self._stopping:
            return
        self.term.write(key.encode())
        self.gui_log(f"Sent '{key}' — {label}.")

    # --- shutdown ladder --------------------------------------------------------
    def request_stop(self, *, already_signalled: bool = False) -> None:
        """Start the shutdown ladder. Safe to call any number of times."""
        with self._stop_lock:
            if self._stopping:
                self.log("Shutdown already in progress — ignoring extra request.")
                return
            self._stopping = True

        if self.stop_button is not None:
            self.stop_button.config(state="disabled", text="Stopping…")
        for b in self.control_buttons:
            b.config(state="disabled")

        if not self._started.is_set():
            # Window closed before anything was launched.
            self.exit_code = 130
            self._finished.set()
            self.messages.put(("close", ""))
            return

        threading.Thread(
            target=self._shutdown_sequence,
            kwargs={"already_signalled": already_signalled},
            daemon=True,
        ).start()

    def _countdown(self, stage: str):
        def tick(remaining: float) -> None:
            self.status(f"{stage} — waiting for clean exit ({remaining:.0f}s left)")

        return tick

    def _shutdown_sequence(self, already_signalled: bool = False) -> None:
        child = self.child
        try:
            if child is None or not child.is_alive():
                self.log("Process already exited.")
            else:
                child.track_descendants()

                if already_signalled:
                    self.log(
                        "Stage 1: SIGINT already delivered by the terminal — "
                        "waiting for LeRobot to finish its cleanup."
                    )
                else:
                    self.log("Stage 1: sending SIGINT (LeRobot graceful stop).")
                    child.signal(signal.SIGINT)

                self.status("Stopping: waiting for LeRobot cleanup…")
                self.log(
                    "Letting LeRobot save the episode, disable torque, close the "
                    "serial port and shut Rerun down."
                )

                if not child.wait(GRACE_COOPERATIVE, self._countdown("Graceful stop")):
                    self.log(
                        f"Still running after {GRACE_COOPERATIVE:.0f}s. "
                        "Stage 2: second SIGINT (KeyboardInterrupt)."
                    )
                    child.signal(signal.SIGINT)

                    if not child.wait(GRACE_INTERRUPT, self._countdown("Interrupt")):
                        self.log(
                            f"Still running after {GRACE_INTERRUPT:.0f}s. Stage 3: SIGTERM."
                        )
                        child.signal(signal.SIGTERM)

                        if not child.wait(GRACE_TERM, self._countdown("Terminate")):
                            self.log(
                                f"Still running after {GRACE_TERM:.0f}s — "
                                "graceful shutdown failed."
                            )
                            child.kill_tree(self.log)

            self._finalize(child)
        finally:
            self._finished.set()
            self.messages.put(("close", ""))

    def _finalize(self, child: ChildProcess | None) -> None:
        if child is None:
            return
        child.reap()
        code = child.returncode
        if code is None:
            code = 0
        elif code < 0:
            self.log(f"Process was killed by signal {-code}.")
        else:
            self.log(f"Process exited with code {code}.")

        self.status("Cleaning up Rerun and leftover processes…")
        child.sweep_survivors(self.log)

        if self.pump is not None:
            self.pump.join()
        self.forwarder.stop()

        # A graceful stop is a successful stop as far as the launcher is concerned;
        # only a forced kill is reported as a failure.
        self.exit_code = 0 if code in (0, -signal.SIGINT) else (code if code > 0 else 1)
        if self.exit_code != 0:
            detail = self.pump.last_error() if self.pump is not None else ""
            self.log(f"FAILED (exit {self.exit_code}): {detail}")
            self.messages.put(("failed", detail))
        else:
            self.log("Shutdown complete.")
            self.status("Shutdown complete.")

    # --- exit paths -------------------------------------------------------------
    def on_child_exited_by_itself(self) -> None:
        with self._stop_lock:
            if self._stopping:
                return
            self._stopping = True
        if self.stop_button is not None:
            self.stop_button.config(state="disabled", text="Finished")
        for b in self.control_buttons:
            b.config(state="disabled")
        self.log("LeRobot finished on its own — cleaning up.")
        threading.Thread(target=self._finish_natural_exit, daemon=True).start()

    def _finish_natural_exit(self) -> None:
        try:
            self._finalize(self.child)
        finally:
            self._finished.set()
            self.messages.put(("close", ""))

    def on_signal(self, signum, _frame) -> None:
        """Terminal Ctrl+C / SIGTERM / terminal closed.

        The child shares our process group, so SIGINT and SIGHUP typed at the terminal
        already reached it; re-sending would burn stage 2 of the ladder immediately.
        """
        name = signal.Signals(signum).name
        self.log(f"Received {name} — starting shutdown.")
        shared = signum in (signal.SIGINT, signal.SIGQUIT, signal.SIGHUP)
        self.request_stop(already_signalled=shared)

    # --- GUI --------------------------------------------------------------------
    def build_gui(self):
        import tkinter as tk
        from tkinter import scrolledtext

        root = tk.Tk()
        root.title(self.title)
        root.geometry("560x520")
        root.minsize(460, 420)
        root.attributes("-topmost", True)

        self.header_var = tk.StringVar(value=self.title)
        tk.Label(
            root, textvariable=self.header_var, font=("TkDefaultFont", 13, "bold"), wraplength=520
        ).pack(pady=(12, 4))

        # --- setup phase: dataset name -----------------------------------------
        self.setup_frame = tk.Frame(root)
        tk.Label(self.setup_frame, text="Dataset name").pack(anchor="w")
        self.name_entry = tk.Entry(self.setup_frame, font=("TkFixedFont", 12))
        self.name_entry.insert(0, self.args.dataset_name_default or "")
        self.name_entry.pack(fill="x", pady=(2, 2))
        self.name_entry.select_range(0, "end")
        self.name_entry.bind("<KeyRelease>", self._check_name)

        self.name_error_var = tk.StringVar(value="")
        tk.Label(self.setup_frame, textvariable=self.name_error_var, fg="#c62828").pack(anchor="w")

        tk.Button(
            self.setup_frame,
            text="Start Recording",
            command=self._submit_name,
            height=2,
            font=("TkDefaultFont", 12, "bold"),
            bg="#2e7d32",
            fg="white",
            activebackground="#1b5e20",
            activeforeground="white",
        ).pack(fill="x", pady=(8, 0))
        root.bind("<Return>", lambda _e: self._submit_name())

        # --- running phase ------------------------------------------------------
        self.running_frame = tk.Frame(root)

        self.episode_var = tk.StringVar(value="")
        tk.Label(
            self.running_frame, textvariable=self.episode_var, font=("TkDefaultFont", 12, "bold")
        ).pack(pady=(0, 2))

        self.status_var = tk.StringVar(value="Starting…")
        tk.Label(
            self.running_frame,
            textvariable=self.status_var,
            wraplength=520,
            justify="center",
            font=("TkDefaultFont", 11),
        ).pack(pady=(0, 2))

        self.progress_var = tk.StringVar(value="")
        tk.Label(self.running_frame, textvariable=self.progress_var, fg="#555").pack(pady=(0, 10))

        if self.args.controls == "record":
            # Start/Pause is one toggle on LeRobot's side ('p'), so it is one button
            # here too; its label follows the state parsed from the recorder's output.
            self.toggle_button = tk.Button(
                self.running_frame,
                text="Start Recording",
                command=self._on_toggle,
                height=2,
                font=("TkDefaultFont", 12, "bold"),
                bg="#2e7d32",
                fg="white",
                activeforeground="white",
            )
            self.toggle_button.pack(fill="x", pady=(0, 6))

            self.finish_button = tk.Button(
                self.running_frame,
                text="Finish & Save Episode",
                command=lambda: self.send_key("n", "finish"),
                height=2,
                font=("TkDefaultFont", 10, "bold"),
                bg="#1565c0",
                fg="white",
                activeforeground="white",
            )
            self.finish_button.pack(fill="x", pady=(0, 6))

            self.discard_button = tk.Button(
                self.running_frame,
                text="Discard & Re-record Episode",
                command=lambda: self.send_key("r", "discard and re-record"),
                height=2,
                font=("TkDefaultFont", 10, "bold"),
                bg="#ef6c00",
                fg="white",
                activeforeground="white",
            )
            self.discard_button.pack(fill="x", pady=(0, 14))

            # Session-level finish, kept visually apart from the per-episode buttons:
            # this one ends the whole run and writes the dataset out.
            tk.Frame(self.running_frame, height=2, bg="#bbb").pack(fill="x", pady=(0, 10))

            self.session_button = tk.Button(
                self.running_frame,
                text="SAVE & FINISH SESSION",
                command=self._on_finish_session,
                height=2,
                font=("TkDefaultFont", 11, "bold"),
                bg="#37474f",
                fg="white",
                activeforeground="white",
            )
            self.session_button.pack(fill="x", pady=(0, 6))

            self.control_buttons = [
                self.toggle_button,
                self.finish_button,
                self.discard_button,
                self.session_button,
            ]

        self.stop_button = tk.Button(
            self.running_frame,
            text="Stop / Exit",
            command=self.request_stop,
            height=2,
            font=("TkDefaultFont", 11, "bold"),
            bg="#c62828",
            fg="white",
            activebackground="#8e0000",
            activeforeground="white",
        )
        self.stop_button.pack(fill="x")

        # --- shared log ---------------------------------------------------------
        self.log_widget = scrolledtext.ScrolledText(
            root, height=10, state="disabled", wrap="word", font=("TkFixedFont", 9)
        )

        # Closing the window is a stop request, not an abandon.
        root.protocol("WM_DELETE_WINDOW", self.request_stop)

        self.root = root
        return root

    # --- per-episode controls ---------------------------------------------------
    def _on_toggle(self) -> None:
        """Start (with countdown) or pause, depending on what the recorder is doing."""
        if self._countdown_left is not None:
            self._cancel_countdown()
            return
        if self.state == "recording":
            self.send_key("p", "pause capture")
        elif self.state in ("armed", "paused"):
            if self.args.countdown > 0:
                self._countdown_left = self.args.countdown
                self._tick_countdown()
            else:
                self.send_key("p", "start capture")

    def _tick_countdown(self) -> None:
        if self._countdown_left is None:
            return
        if self._countdown_left <= 0:
            self._countdown_left = None
            self._countdown_job = None
            self.status_var.set("Go!")
            self.send_key("p", "start capture")
            return
        self.status_var.set(f"Recording starts in {self._countdown_left}…")
        self.toggle_button.config(text=f"Cancel countdown ({self._countdown_left})")
        self._countdown_left -= 1
        self._countdown_job = self.root.after(1000, self._tick_countdown)

    def _cancel_countdown(self) -> None:
        if self._countdown_job is not None:
            self.root.after_cancel(self._countdown_job)
            self._countdown_job = None
        self._countdown_left = None
        self.status_var.set("Countdown cancelled — armed.")
        self._apply_state()

    def _on_finish_session(self) -> None:
        """End the whole run: LeRobot saves the dataset and exits cleanly."""
        self._cancel_countdown() if self._countdown_left is not None else None
        self.status_var.set("Finishing session — saving dataset…")
        self.send_key("q", "save and finish the session")

    def _set_state(self, state: str) -> None:
        if state != self.state and self._countdown_left is not None:
            # The recorder moved on by itself (or via the terminal); drop the countdown.
            self._cancel_countdown()
        self.state = state
        self._apply_state()

    def _apply_state(self) -> None:
        """Enable/relabel the buttons to match the recorder's current phase."""
        if self.args.controls != "record" or self._stopping:
            return

        state = self.state
        can_capture = state in ("armed", "paused", "recording")

        if state == "recording":
            self.toggle_button.config(text="Pause Recording", bg="#ef6c00", state="normal")
        elif state in ("armed", "paused"):
            label = "Start Recording" if state == "armed" else "Resume Recording"
            self.toggle_button.config(text=label, bg="#2e7d32", state="normal")
        else:
            self.toggle_button.config(text="Start Recording", bg="#2e7d32", state="disabled")

        self.finish_button.config(
            text="Finish Reset" if state == "reset" else "Finish & Save Episode",
            state="normal" if (can_capture or state == "reset") else "disabled",
        )
        self.discard_button.config(state="normal" if can_capture else "disabled")
        self.session_button.config(state="normal" if state != "stopping" else "disabled")

    def _dataset_path(self, name: str) -> str | None:
        if not self.args.dataset_dir or not name:
            return None
        return os.path.join(os.path.expanduser(self.args.dataset_dir), name)

    def _free_name(self, name: str) -> str:
        """First unused name near *name*: pnp10 -> pnp11, run -> run_2."""
        m = re.match(r"^(.*?)(\d+)$", name)
        stem, n, width = (m.group(1), int(m.group(2)) + 1, len(m.group(2))) if m else (name + "_", 2, 0)
        for _ in range(1000):
            candidate = f"{stem}{n:0{width}d}" if width else f"{stem}{n}"
            path = self._dataset_path(candidate)
            if path is None or not os.path.exists(path):
                return candidate
            n += 1
        return name

    def _check_name(self, _event=None) -> None:
        """Live feedback while typing, so the clash is obvious before pressing Start."""
        if self.name_entry is None or self._started.is_set():
            return
        name = self.name_entry.get().strip()
        path = self._dataset_path(name)
        if path and os.path.exists(path):
            self.name_error_var.set(f"'{name}' already exists — pick another name.")
        elif name and not NAME_RE.match(name):
            self.name_error_var.set("Letters, digits, dot, dash, underscore; start alphanumeric.")
        else:
            self.name_error_var.set("")

    def _submit_name(self) -> None:
        if self._started.is_set() or self.name_entry is None:
            return
        name = self.name_entry.get().strip()
        if not NAME_RE.match(name):
            self.name_error_var.set(
                "Use letters, digits, dot, dash or underscore; must start alphanumeric."
            )
            return

        # LeRobot creates the dataset directory with exist_ok=False, so an existing
        # name aborts the run only after Rerun and the arm are already up. Catch it here.
        path = self._dataset_path(name)
        if path and os.path.exists(path):
            suggestion = self._free_name(name)
            self.name_error_var.set(f"'{name}' already exists. Suggested: '{suggestion}'.")
            self.name_entry.delete(0, "end")
            self.name_entry.insert(0, suggestion)
            self.name_entry.select_range(0, "end")
            return

        self.name_error_var.set("")
        self.setup_frame.pack_forget()
        self._show_running(f"{self.title} — {name}")
        self.launch(name)

    def _show_running(self, header: str) -> None:
        self.header_var.set(header)
        self.running_frame.pack(fill="x", padx=16, pady=(0, 10))
        self.log_widget.pack(fill="both", expand=True, padx=12, pady=(0, 12))

    def _show_failure(self, detail: str) -> None:
        """Leave the window up with the error, so it is not lost in scrollback."""
        self._failed = True
        if self.status_var is not None:
            self.status_var.set(f"FAILED (exit {self.exit_code})")
        if self.episode_var is not None:
            self.episode_var.set("Recording failed")
        if self.progress_var is not None:
            self.progress_var.set(detail[:300])
        for b in self.control_buttons:
            b.config(state="disabled")
        if self.stop_button is not None:
            self.stop_button.config(
                text="Close", state="normal", bg="#37474f", command=self.root.destroy
            )
        if self.root is not None:
            self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)

    def _append_log(self, message: str) -> None:
        if self.log_widget is None:
            return
        self.log_widget.config(state="normal")
        self.log_widget.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.log_widget.see("end")
        self.log_widget.config(state="disabled")

    def _pump(self) -> None:
        """Drain the worker queue and poll the child. Tk-thread only."""
        while True:
            try:
                kind, payload = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._append_log(str(payload))
            elif kind == "status" and self.status_var is not None:
                if self._countdown_left is None:  # never stomp on a live countdown
                    self.status_var.set(str(payload))
            elif kind == "episode" and self.episode_var is not None:
                self.episode_var.set(str(payload))
            elif kind == "progress" and self.progress_var is not None:
                self.progress_var.set(str(payload))
            elif kind == "state":
                self._set_state(str(payload))
            elif kind == "failed":
                self._show_failure(str(payload))
            elif kind == "close":
                if not self._failed:
                    self.root.after(600, self.root.destroy)

        if self._started.is_set() and self.child is not None:
            if not self._stopping and not self.child.is_alive():
                self.on_child_exited_by_itself()
            elif not self._stopping:
                self.child.track_descendants()

        if self.root is not None:
            self.root.after(200, self._pump)

    def run_gui(self) -> int:
        root = self.build_gui()

        if self.args.prompt_dataset_name:
            self.setup_frame.pack(fill="x", padx=16, pady=(0, 10))
            self.name_entry.focus_set()
        else:
            self._show_running(self.title)
            self.launch(None)

        root.after(200, self._pump)
        # Drop always-on-top after the window has been placed, so it does not sit over
        # the Rerun viewer for the whole session.
        root.after(2500, lambda: root.attributes("-topmost", False))
        root.mainloop()
        self._finished.wait(timeout=30)
        self.forwarder.stop()
        self.term.close()
        return self.exit_code

    def run_headless(self) -> int:
        self.log("No usable display — running without the GUI (Ctrl+C still stops cleanly).")
        if not self._started.is_set():
            name = self.args.dataset_name_default if self.args.prompt_dataset_name else None
            self.launch(name)
        while self.child.is_alive():
            self.child.track_descendants()
            time.sleep(POLL_INTERVAL)
        if not self._stopping:
            self.on_child_exited_by_itself()
        self._finished.wait(timeout=GRACE_COOPERATIVE + GRACE_INTERRUPT + GRACE_TERM + 60)
        self.forwarder.stop()
        self.term.close()
        return self.exit_code


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a LeRobot command with a graceful control/Stop GUI."
    )
    parser.add_argument("--title", default="OMX Control", help="Window title.")
    parser.add_argument(
        "--controls",
        choices=("none", "record"),
        default="none",
        help="Show the per-episode recording buttons.",
    )
    parser.add_argument(
        "--prompt-dataset-name",
        action="store_true",
        help="Ask for a dataset name first and substitute it for {DATASET} in the command.",
    )
    parser.add_argument("--dataset-name-default", default="", help="Pre-filled dataset name.")
    parser.add_argument(
        "--dataset-dir",
        default="",
        help="Directory datasets are created in; used to reject names that already exist.",
    )
    parser.add_argument(
        "--countdown",
        type=int,
        default=3,
        help="Seconds to count down before capture starts (0 to start immediately).",
    )
    parser.add_argument(
        "--dataset-name-out", default="", help="File to write the chosen dataset name to."
    )
    parser.add_argument(
        "command", nargs=argparse.REMAINDER, help="Command to supervise, after a '--' separator."
    )
    args = parser.parse_args()

    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if not args.command:
        parser.error("no command given (use: omx_supervisor.py [options] -- <command>)")

    supervisor = Supervisor(args)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, supervisor.on_signal)
        except (ValueError, OSError):
            pass

    try:
        return supervisor.run_gui()
    except Exception as e:  # Tk unavailable / no display
        print(f"[stop-gui] GUI unavailable ({e}).", flush=True)
        return supervisor.run_headless()
    finally:
        supervisor.forwarder.stop()


if __name__ == "__main__":
    sys.exit(main())
