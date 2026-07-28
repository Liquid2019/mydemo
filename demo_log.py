# -*- coding: utf-8 -*-
"""Demo 日志：写入 logs/stageN_*.log，同时保留 print 输出。"""

import atexit
import os
import sys
import traceback
from datetime import datetime


class _Tee:
    def __init__(self, stream, log_fp):
        self._stream = stream
        self._log = log_fp

    def write(self, data):
        self._stream.write(data)
        if self._log and not self._log.closed:
            self._log.write(data)
            self._log.flush()

    def flush(self):
        self._stream.flush()
        if self._log and not self._log.closed:
            self._log.flush()


_log_fp = None
_log_path = None
_orig_stdout = None
_orig_stderr = None


def setup_demo_log(stage: str, script_dir=None):
    """Redirect stdout/stderr to logs/stageN_timestamp.log + stageN_latest.log."""
    global _log_fp, _log_path, _orig_stdout, _orig_stderr
    if _log_fp is not None:
        return _log_path

    base = script_dir or os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(base, "logs")
    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(log_dir, f"{stage}_{stamp}.log")
    latest = os.path.join(log_dir, f"{stage}_latest.log")

    fp = open(log_path, "a", encoding="utf-8", buffering=1)
    fp.write(f"=== {stage} start {datetime.now().isoformat()} ===\n")
    fp.flush()

    try:
        if os.path.islink(latest) or os.path.isfile(latest):
            os.remove(latest)
        os.symlink(os.path.basename(log_path), latest)
    except OSError:
        pass

    _orig_stdout = sys.stdout
    _orig_stderr = sys.stderr
    sys.stdout = _Tee(_orig_stdout, fp)
    sys.stderr = _Tee(_orig_stderr, fp)
    _log_fp = fp
    _log_path = log_path
    atexit.register(_close_log)
    return log_path


def _close_log():
    global _log_fp, _orig_stdout, _orig_stderr
    if _log_fp is None:
        return
    try:
        _log_fp.write(f"=== exit {datetime.now().isoformat()} ===\n")
        _log_fp.flush()
        _log_fp.close()
    except Exception:
        pass
    if _orig_stdout is not None:
        sys.stdout = _orig_stdout
    if _orig_stderr is not None:
        sys.stderr = _orig_stderr
    _log_fp = None


def log_exception(prefix="FATAL"):
    tb = traceback.format_exc()
    print(f"[{prefix}]\n{tb}", file=sys.stderr)
    return tb
