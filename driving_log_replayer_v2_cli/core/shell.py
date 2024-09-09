from pathlib import Path
import subprocess
import sys


def run_with_log(cmd: list, log_path: Path) -> None:
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    f = log_path.open("w", encoding="utf-8")

    try:
        while True:
            line = proc.stdout.readline().decode("utf-8")
            sys.stdout.write(line)
            f.write(line)
            if not line and proc.poll() is not None:
                break
    finally:
        f.close()
