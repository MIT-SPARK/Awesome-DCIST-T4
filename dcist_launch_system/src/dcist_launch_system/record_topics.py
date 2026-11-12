#!/usr/bin/env python3
"""Wrap ``ros2 bag record`` with the topics listed in a file.

Topic file syntax (recording/*_topics.txt):

* one topic per line; ``$VAR`` / ``${VAR}`` are expanded from the environment
* ``#`` starts a comment (whole line or trailing); blank lines are ignored
* a trailing ``# optional`` marks a topic that may legitimately stay silent during a
  mission (e.g. pick_confirmation). It is recorded like any other; the marker is for
  checks that expect a message on every required topic (``parse_topic_file``)
* ``*`` matches one name segment (``/*/omniplanner_node/compiled_plan_out``: every
  robot's plan). With any ``*`` line the whole list is recorded with one anchored
  ``--regex`` instead of ``--topics``.

Clean stop: the recorder runs in its own process group and this wrapper turns
SIGINT, SIGTERM and SIGHUP into one SIGINT to it, then waits for it to exit. Ctrl-C in
the tmux pane and ``tmux kill-session`` (SIGHUP) both close the bag with its
metadata.yaml written; nothing kills the recorder after a timeout.

One recorder per lock name (``--lock NAME``, e.g. ``base``): an exclusive flock on
``~/.cache/adt4/mission_bag_<NAME>.lock``, held by the recorder for as long as it runs.
A second recorder with the same name (e.g. the base_station_console session and a
base_station_* session on the same machine) waits until the first one stops, then
records into its own output: never two at once, and no gap when the first session
ends before the second.
"""

import fcntl
import os
import re
import signal
import subprocess
import sys

import click


def parse_topic_file(path, expand=True):
    """``[(topic, optional)]`` from a topic file (see module doc)."""
    out = []
    with open(path, "r") as fin:
        for line in fin:
            topic, _, comment = line.partition("#")
            topic = topic.strip()
            if expand:
                topic = os.path.expandvars(topic)
            if not topic:
                continue
            optional = comment.strip().lower().startswith("optional")
            out.append((topic, optional))
    return out


def topics_regex(topics):
    """One anchored regex matching every topic (``*`` = one name segment)."""
    parts = [re.escape(t).replace(r"\*", "[^/]+") for t in topics]
    return "^(" + "|".join(parts) + ")$"


def build_command(
    output, topics, storage=None, preset=None, max_duration=None, max_size=None
):
    cmd = ["ros2", "bag", "record", "-o", str(output)]
    if storage:
        cmd += ["-s", storage]
    if preset:
        cmd += ["--storage-preset-profile", preset]
    if max_duration:
        cmd += ["--max-bag-duration", str(int(max_duration))]
    if max_size:
        cmd += ["--max-bag-size", str(int(max_size))]
    if any("*" in t for t in topics):
        cmd += ["--regex", topics_regex(topics)]
    else:
        cmd += ["--topics"] + list(topics)
    return cmd


STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def _default_signals():
    # a shell starts background jobs with SIGINT ignored, and an ignored signal stays
    # ignored across exec: the recorder must see SIGINT with its default disposition
    for sig in STOP_SIGNALS:
        signal.signal(sig, signal.SIG_DFL)


def lock_path(name):
    return os.path.join(
        os.path.expanduser("~"), ".cache", "adt4", f"mission_bag_{name}.lock"
    )


def acquire_lock(name, output, wait=True, log=print):
    """An fd holding the exclusive lock ``name`` (blocks while another holds it)."""
    path = lock_path(name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        try:
            holder = os.pread(fd, 4096, 0).decode(errors="replace").strip()
        except OSError:
            holder = ""
        if not wait:
            os.close(fd)
            return None
        log(
            f"Another mission bag '{name}' is recording on this machine "
            f"({holder or 'unknown'}); waiting for it to stop before recording",
            flush=True,
        )
        fcntl.flock(fd, fcntl.LOCK_EX)
    os.ftruncate(fd, 0)
    os.pwrite(fd, f"pid {os.getpid()} -> {output}\n".encode(), 0)
    return fd


def supervise(cmd, pass_fds=()):
    """Run ``cmd`` in its own process group; forward stop signals as SIGINT; wait."""
    child = []
    pending = []

    def forward(signum, frame):
        if not child:
            pending.append(signum)
            return
        try:
            child[0].send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

    for sig in STOP_SIGNALS:
        signal.signal(sig, forward)
    child.append(
        subprocess.Popen(
            cmd,
            start_new_session=True,
            preexec_fn=_default_signals,
            pass_fds=tuple(pass_fds),
        )
    )
    if pending:
        forward(pending[0], None)
    return child[0].wait()


@click.command
@click.argument("output", type=click.Path())
@click.argument("topic_file", type=click.Path(exists=True))
@click.option("--storage", "-s", default=None, help="rosbag2 storage (mcap, sqlite3)")
@click.option(
    "--preset",
    default=None,
    help="storage preset profile, e.g. zstd_fast (mcap chunk compression)",
)
@click.option(
    "--max-bag-duration", type=int, default=None, help="split every N seconds"
)
@click.option("--max-bag-size", type=int, default=None, help="split every N bytes")
@click.option(
    "--lock",
    default=None,
    help="one recorder per NAME on this machine; a second one waits (see module doc)",
)
@click.option("--dry-run", is_flag=True, help="print the command only")
def main(
    output, topic_file, storage, preset, max_bag_duration, max_bag_size, lock, dry_run
):
    """Wrap rosbag record with topics from file"""
    topics = [t for t, _ in parse_topic_file(topic_file)]
    if not topics:
        sys.exit(f"no topics in {topic_file}")
    cmd = build_command(output, topics, storage, preset, max_bag_duration, max_bag_size)
    print(f"Running: '{' '.join(cmd)}'", flush=True)
    if dry_run:
        return
    fds = ()
    if lock:
        # the recorder inherits the lock: it stays held while the bag is open, even
        # if this wrapper dies first
        fds = (acquire_lock(lock, output),)
        print(f"Recording (lock '{lock}' held)", flush=True)
    sys.exit(supervise(cmd, pass_fds=fds))


if __name__ == "__main__":
    main()
