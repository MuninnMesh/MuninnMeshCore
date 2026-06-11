"""Expose the current Git revision to MuziWorks builds."""

import subprocess

Import("env", "projenv")  # pylint: disable=undefined-variable

# Always run git against the project repo, not the process CWD — `pio run -d`
# from elsewhere would otherwise stamp the splash with the wrong repo's hash.
PROJECT_DIR = env.subst("$PROJECT_DIR")


def _git_output(args):
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.DEVNULL,
                                       cwd=PROJECT_DIR).strip()
    except Exception:  # pylint: disable=broad-except
        return ""


def _git_call(args):
    try:
        return subprocess.call(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               cwd=PROJECT_DIR)
    except Exception:  # pylint: disable=broad-except
        return -1


git_hash = _git_output(["git", "rev-parse", "--short=8", "HEAD"]) or "unknown"
dirty = (
    _git_call(["git", "diff", "--quiet"]) != 0
    or _git_call(["git", "diff", "--cached", "--quiet"]) != 0
)
if git_hash != "unknown" and dirty:
    git_hash = f"{git_hash}+"

macro_define = [("MUNINN_CODE_GIT_HASH", env.StringifyMacro(git_hash))]
# PlatformIO compiles some sources with env and others through projenv; append
# to both so the boot splash macro is visible across all build contexts.
env.Append(CPPDEFINES=macro_define)
projenv.Append(CPPDEFINES=macro_define)
