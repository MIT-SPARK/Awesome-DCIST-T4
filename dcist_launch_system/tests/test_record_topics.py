"""record_topics: the topic file of the stack's ``recording`` window."""

from click.testing import CliRunner

from dcist_launch_system import record_topics


def _run(tmp_path, monkeypatch, text, robots):
    ran = []
    monkeypatch.setattr(record_topics.subprocess, "run", ran.append)
    monkeypatch.setenv("ADT4_ROBOT_NAME", "willow")
    if robots is None:
        monkeypatch.delenv("ADT4_FLEET_ROBOTS", raising=False)
    else:
        monkeypatch.setenv("ADT4_FLEET_ROBOTS", robots)
    topics = tmp_path / "topics.txt"
    topics.write_text(text)
    result = CliRunner().invoke(record_topics.main, ["out", str(topics)])
    return result, ran


def test_a_fleet_line_is_one_topic_per_robot(tmp_path, monkeypatch):
    result, ran = _run(
        tmp_path,
        monkeypatch,
        "/${ADT4_ROBOT_NAME}/tf\n/${ADT4_FLEET_ROBOT}/manual_mode\n",
        "euclid, gauss",
    )
    assert result.exit_code == 0, result.output
    assert ran[0][-3:] == ["/willow/tf", "/euclid/manual_mode", "/gauss/manual_mode"]


def test_a_fleet_line_without_the_fleet_is_refused(tmp_path, monkeypatch):
    result, ran = _run(
        tmp_path, monkeypatch, "/${ADT4_FLEET_ROBOT}/manual_mode\n", None
    )
    assert result.exit_code != 0 and "ADT4_FLEET_ROBOTS" in result.output
    assert ran == []  # nothing is recorded with a guessed fleet


def test_a_file_without_fleet_lines_needs_no_fleet(tmp_path, monkeypatch):
    result, ran = _run(tmp_path, monkeypatch, "/${ADT4_ROBOT_NAME}/tf\n", None)
    assert result.exit_code == 0 and ran[0][-1] == "/willow/tf"
