"""Expose the current Git revision to MuziWorks builds."""

import subprocess

Import("env", "projenv")  # pylint: disable=undefined-variable


def _git_output(args):
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:  # pylint: disable=broad-except
        return ""


git_hash = _git_output(["git", "rev-parse", "--short=8", "HEAD"]) or "unknown"
dirty = (
    subprocess.call(["git", "diff", "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0
    or subprocess.call(["git", "diff", "--cached", "--quiet"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0
)
if git_hash != "unknown" and dirty:
    git_hash = f"{git_hash}+"

macro_define = [("MUNINN_CODE_GIT_HASH", env.StringifyMacro(git_hash))]
# PlatformIO compiles some sources with env and others through projenv; append
# to both so the boot splash macro is visible across all build contexts.
env.Append(CPPDEFINES=macro_define)
projenv.Append(CPPDEFINES=macro_define)
