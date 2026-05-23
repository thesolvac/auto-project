"""Tests for the JSONL run logger."""

from autoproject.utils.logger import RunLogger, new_run_dir, read_run


def test_logger_writes_and_reads_back(tmp_path):
    with RunLogger(tmp_path, run_id="r1") as logger:
        logger.log({"t": 0.0, "x": 1})
        logger.log({"t": 0.1, "x": 2})
    records = read_run(tmp_path / "r1")
    assert records == [{"t": 0.0, "x": 1}, {"t": 0.1, "x": 2}]


def test_read_run_accepts_file_or_dir(tmp_path):
    logger = RunLogger(tmp_path, run_id="r2")
    logger.log({"a": 1})
    logger.close()
    assert read_run(tmp_path / "r2") == read_run(tmp_path / "r2" / "run.jsonl")


def test_new_run_dir_creates_directory(tmp_path):
    d = new_run_dir(tmp_path, run_id="abc")
    assert d.is_dir()
    assert d == tmp_path / "abc"
