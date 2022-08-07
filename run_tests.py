# https://tbrink.science/blog/2017/04/30/processing-the-output-of-a-subprocess-with-python-in-realtime/

import subprocess
import os
import errno
import pty
import select
from threading import Timer
import sys


class OutStream:
    def __init__(self, fileno):
        self._fileno = fileno
        self._buffer = b""

    def read_lines(self):
        try:
            output = os.read(self._fileno, 1000)
        except OSError as e:
            if e.errno != errno.EIO:
                raise
            output = b""
        lines = output.split(b"\n")
        lines[0] = self._buffer + lines[0]  # prepend previous
        # non-finished line.
        if output:
            self._buffer = lines[-1]
            finished_lines = lines[:-1]
            readable = True
        else:
            self._buffer = b""
            if len(lines) == 1 and not lines[0]:
                # We did not have buffer left, so no output at all.
                lines = []
            finished_lines = lines
            readable = False
        finished_lines = [line.rstrip(b"\r").decode()
                          for line in finished_lines]
        return finished_lines, readable

    def fileno(self):
        return self._fileno


# Start the subprocess.
out_r, out_w = pty.openpty()
err_r, err_w = pty.openpty()
proc = subprocess.Popen(["pros", "t"], stdout=out_w, stderr=err_w)
os.close(out_w)  # if we do not write to process, close these.
os.close(err_w)


def cancel():
    print("cancelling command (timed out)")
    proc.kill()


timeout_s = 10
t = Timer(timeout_s, cancel)
t.start()
fds = {OutStream(out_r), OutStream(err_r)}
all_lines = []
while fds:
    # Call select(), anticipating interruption by signals.
    while True:
        try:
            rlist, _, _ = select.select(fds, [], [])
            break
        except InterruptedError:
            continue
    # Handle all file descriptors that are ready.
    for f in rlist:
        lines, readable = f.read_lines()
        for line in lines:
            t.cancel()
            t = Timer(timeout_s, cancel)
            t.start()
            print(line)
            all_lines.append(line)
        if not readable:
            # This OutStream is finished.
            fds.remove(f)

t.cancel()
last_line = lines[:-1]
if last_line == "OK":
    sys.exit(0)
else:
    sys.exit(1)
