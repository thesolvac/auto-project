"""Tests for autoproject.utils.config — YAML loading and provisional warnings."""

import logging
from pathlib import Path

import pytest
from pydantic import BaseModel, ValidationError

from autoproject.utils.config import CONFIG_DIR, load_config


def _write(tmp_path: Path, text: str, name: str = "cfg.yaml") -> Path:
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def test_plain_config_returns_dict_without_warning(tmp_path, caplog):
    path = _write(tmp_path, "a: 1\nb:\n  c: 2\n")
    with caplog.at_level(logging.WARNING):
        cfg = load_config(path)
    assert cfg == {"a": 1, "b": {"c": 2}}
    assert caplog.records == []


def test_provisional_true_warns_for_whole_file(tmp_path, caplog):
    path = _write(tmp_path, "provisional: true\nx: 5\n")
    with caplog.at_level(logging.WARNING):
        cfg = load_config(path)
    assert "provisional" not in cfg  # marker is stripped from the result
    assert len(caplog.records) == 1
    assert "ALL values are provisional" in caplog.records[0].getMessage()


def test_provisional_list_warns_per_path(tmp_path, caplog):
    text = "provisional:\n  - drive.wheelbase\n  - gear\ndrive:\n  wheelbase: 0.18\ngear: 1.0\n"
    path = _write(tmp_path, text)
    with caplog.at_level(logging.WARNING):
        load_config(path)
    messages = [r.getMessage() for r in caplog.records]
    assert len(messages) == 2
    assert any("drive.wheelbase" in m for m in messages)
    assert any("gear" in m for m in messages)


def test_provisional_false_is_silent(tmp_path, caplog):
    path = _write(tmp_path, "provisional: false\nx: 1\n")
    with caplog.at_level(logging.WARNING):
        load_config(path)
    assert caplog.records == []


def test_stale_provisional_path_raises(tmp_path):
    path = _write(tmp_path, "provisional:\n  - drive.missing\ndrive:\n  wheelbase: 0.18\n")
    with pytest.raises(ValueError, match="does not exist"):
        load_config(path)


def test_bad_provisional_type_raises(tmp_path):
    path = _write(tmp_path, "provisional: 5\nx: 1\n")
    with pytest.raises(ValueError, match="must be a bool or a list"):
        load_config(path)


def test_non_mapping_top_level_raises(tmp_path):
    path = _write(tmp_path, "- 1\n- 2\n")
    with pytest.raises(ValueError, match="must be a mapping"):
        load_config(path)


def test_empty_file_returns_empty_dict(tmp_path):
    path = _write(tmp_path, "")
    assert load_config(path) == {}


def test_pydantic_model_validation(tmp_path):
    class Drive(BaseModel):
        wheelbase_m: float
        gear_ratio: float

    class Params(BaseModel):
        drive: Drive

    path = _write(tmp_path, "drive:\n  wheelbase_m: 0.18\n  gear_ratio: 1.0\n")
    cfg = load_config(path, model=Params)
    assert isinstance(cfg, Params)
    assert cfg.drive.wheelbase_m == 0.18

    bad_text = "drive:\n  wheelbase_m: not_a_number\n  gear_ratio: 1.0\n"
    bad = _write(tmp_path, bad_text, name="bad.yaml")
    with pytest.raises(ValidationError):
        load_config(bad, model=Params)


@pytest.mark.parametrize("yaml_file", sorted(CONFIG_DIR.rglob("*.yaml")), ids=lambda p: p.name)
def test_real_config_files_load(yaml_file):
    """Every shipped config file parses and its provisional markers resolve."""
    cfg = load_config(yaml_file)
    assert isinstance(cfg, dict)


def test_robot_params_fully_measured(caplog):
    # All kinematic values are now measured/derived -> no provisional warnings.
    with caplog.at_level(logging.WARNING):
        load_config(CONFIG_DIR / "robot_params.yaml")
    assert caplog.records == []


def test_hardware_pins_marked_provisional(caplog):
    # Pins are assumptions until the wiring is confirmed -> whole-file warning.
    with caplog.at_level(logging.WARNING):
        load_config(CONFIG_DIR / "hardware_pins.yaml")
    messages = [r.getMessage() for r in caplog.records]
    assert any("ALL values are provisional" in m for m in messages)
