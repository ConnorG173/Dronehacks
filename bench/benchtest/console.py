"""bench.console - the operator's terminal over SSH: keys, signals, safe printing.

There is no RC transmitter, so the SSH terminal IS the operator's control:

    space / l     land now
    k             KILL (force-disarm: motors stop at once, the vehicle drops)
    Ctrl-C        land; a second Ctrl-C kills
    SSH drops     the shell's SIGHUP -> land (sshd must notice a dead link quickly:
                  see README.md, ClientAliveInterval)
    stdin closes  land

Printing never raises: after the SSH link drops, writing to the dead terminal fails with
EIO, and that must not stop the landing.
"""
from __future__ import annotations

import os
import select
import signal
import sys
import threading

try:
    import termios
    import tty
except ImportError:          # not a POSIX terminal (only matters for the key thread)
    termios = tty = None


def say(*parts, end="\n"):
    try:
        sys.stdout.write(" ".join(str(p) for p in parts) + end)
        sys.stdout.flush()
    except (OSError, ValueError):
        pass


def status_line(text):
    say("\r\033[K" + text, end="")


class Operator:
    """Collects land / kill requests from keys and signals.  Poll `land` and `kill`."""

    def __init__(self):
        self.land = None             # reason string once requested
        self.kill = None
        self._raw = False
        self._old = None
        self._stop = threading.Event()
        self._thread = None
        self._sigint_count = 0
        self._old_handlers = {}

    # --------------------------------------------------------------- signals
    def install_signals(self):
        for sig, fn in ((signal.SIGINT, self._on_sigint), (signal.SIGHUP, self._on_hup),
                        (signal.SIGTERM, self._on_term)):
            self._old_handlers[sig] = signal.signal(sig, fn)

    def restore_signals(self):
        for sig, old in self._old_handlers.items():
            signal.signal(sig, old)
        self._old_handlers.clear()

    def _on_sigint(self, *_):
        self._sigint_count += 1
        if self._sigint_count == 1:
            self.request_land("Ctrl-C (press Ctrl-C again to KILL)")
        else:
            self.request_kill("second Ctrl-C")

    def _on_hup(self, *_):
        self.request_land("SSH session lost (SIGHUP)")

    def _on_term(self, *_):
        self.request_land("SIGTERM")

    def request_land(self, why):
        if self.land is None:
            self.land = why
            say(f"\n>>> LAND requested: {why}")

    def request_kill(self, why):
        if self.kill is None:
            self.kill = why
            say(f"\n>>> KILL requested: {why}")

    # --------------------------------------------------------------- keys
    def start_keys(self):
        """Raw single-key input in a background thread (only while flying)."""
        if termios is None or not sys.stdin.isatty():
            say("(stdin is not a terminal: keys disabled; Ctrl-C / SIGHUP / SIGTERM still land)")
            return
        fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(fd)
        tty.setcbreak(fd)            # keys without Enter; Ctrl-C still raises SIGINT
        self._raw = True
        self._thread = threading.Thread(target=self._keys, args=(fd,), daemon=True)
        self._thread.start()

    def stop_keys(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._raw and self._old is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old)
            except (OSError, termios.error):
                pass
        self._raw = False

    def _keys(self, fd):
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue
                ch = os.read(fd, 1)
            except OSError:
                self.request_land("terminal lost")
                return
            if not ch:
                self.request_land("stdin closed")
                return
            c = ch.decode(errors="ignore").lower()
            if c in (" ", "l"):
                self.request_land("operator key")
            elif c == "k":
                self.request_kill("operator key k")

    def __enter__(self):
        self.install_signals()
        return self

    def __exit__(self, *exc):
        self.stop_keys()
        self.restore_signals()
        return False


def confirm(prompt, expected):
    """Ask the operator to type `expected` exactly. False on anything else (including EOF)."""
    try:
        ans = input(f"{prompt}\n  type {expected!r} to continue, anything else aborts: ")
    except (EOFError, KeyboardInterrupt):
        say()
        return False
    return ans.strip() == expected


def ask(prompt, default=""):
    try:
        return input(prompt).strip() or default
    except (EOFError, KeyboardInterrupt):
        say()
        return None


def in_multiplexer():
    """tmux / screen keep the program alive when SSH drops, which defeats the link-loss landing."""
    return bool(os.environ.get("TMUX") or os.environ.get("STY"))
