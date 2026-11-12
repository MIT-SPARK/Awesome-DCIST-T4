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
"""

import os
import re
import signal
import subprocess
import sys

import click


def parse_topic_file(path):
    """``[(topic, optional)]`` from a topic file (see module doc)."""
    out = []
    with open(path, "r") as fin:
        for line in fin:
            topic, _, comment = line.partition("#")
            topic = os.path.expandvars(topic.strip())
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


def supervise(cmd):
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
        subprocess.Popen(cmd, start_new_session=True, preexec_fn=_default_signals)
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
@click.option("--dry-run", is_flag=True, help="print the command only")
def main(output, topic_file, storage, preset, max_bag_duration, max_bag_size, dry_run):
    """Wrap rosbag record with topics from file"""
    topics = [t for t, _ in parse_topic_file(topic_file)]
    if not topics:
        sys.exit(f"no topics in {topic_file}")
    cmd = build_command(output, topics, storage, preset, max_bag_duration, max_bag_size)
    print(f"Running: '{' '.join(cmd)}'", flush=True)
    if dry_run:
        return
    sys.exit(supervise(cmd))


if __name__ == "__main__":
    main()
