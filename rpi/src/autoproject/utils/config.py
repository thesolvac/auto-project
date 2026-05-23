"""Configuration loading with provisional-value tracking.

Config files live in ``config/*.yaml`` and are loaded once at startup. A file may
declare a top-level ``provisional`` marker for values that are placeholders
awaiting real measurement (e.g. on the assembled robot in Phase 7):

- ``provisional: true`` — every value in the file is unconfirmed.
- ``provisional: ["drive.wheelbase_m", "intrinsics"]`` — only the listed dotted
  field paths are unconfirmed.

At load time each provisional value is logged as a ``WARNING`` so unmeasured
placeholders are never silently trusted. Once a value is measured, removing it
from the ``provisional`` list (or deleting the marker) silences the warning.

Optionally a ``pydantic`` model can be passed to validate and coerce the parsed
data — the project standard for configuration validation.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, TypeVar

import yaml

if TYPE_CHECKING:
    from pydantic import BaseModel

logger = logging.getLogger(__name__)

# Repo-root ``config/`` directory, resolved relative to this file:
#   .../rpi/src/autoproject/utils/config.py  ->  parents[4] == repo root
CONFIG_DIR = Path(__file__).resolve().parents[4] / "config"

# Top-level YAML key carrying the provisional marker (bool or list of paths).
_PROVISIONAL_KEY = "provisional"

ModelT = TypeVar("ModelT", bound="BaseModel")


def _resolve_path(data: dict[str, Any], dotted: str) -> tuple[bool, Any]:
    """Resolve a dotted path (``a.b.c``) within nested dicts.

    Returns ``(found, value)``; ``found`` is False if any segment is missing.
    """
    cursor: Any = data
    for segment in dotted.split("."):
        if isinstance(cursor, dict) and segment in cursor:
            cursor = cursor[segment]
        else:
            return False, None
    return True, cursor


def _warn_provisional(data: dict[str, Any], marker: Any, source: str) -> None:
    """Emit a warning for each provisional value declared by ``marker``.

    Raises ``ValueError`` if a listed dotted path does not exist in ``data`` (a
    stale or mistyped marker), or if the marker is neither a bool nor a list.
    """
    if marker is True:
        logger.warning(
            "config %s: ALL values are provisional placeholders; "
            "measure/confirm before relying on them",
            source,
        )
        return
    if marker is False:
        return
    if not isinstance(marker, list):
        raise ValueError(
            f"{source}: '{_PROVISIONAL_KEY}' must be a bool or a list of dotted "
            f"field paths, got {type(marker).__name__}"
        )

    for dotted in marker:
        found, value = _resolve_path(data, dotted)
        if not found:
            raise ValueError(
                f"{source}: provisional path '{dotted}' does not exist in the "
                f"config (stale or mistyped marker)"
            )
        logger.warning(
            "config %s: '%s' is provisional (= %r); measure before relying on it",
            source,
            dotted,
            value,
        )


def load_config(path: str | Path, model: type[ModelT] | None = None) -> dict[str, Any] | ModelT:
    """Load and validate a YAML config file.

    Parses ``path``, emits a warning for every value marked ``provisional``, and
    strips the marker from the returned data. If ``model`` is given, the parsed
    data is validated and coerced through ``model.model_validate`` and the model
    instance is returned; otherwise the plain ``dict`` is returned.

    Example::

        params = load_config(CONFIG_DIR / "robot_params.yaml")
        wheelbase = params["drive"]["wheelbase_m"]
    """
    path = Path(path)
    parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
    if parsed is None:
        parsed = {}
    if not isinstance(parsed, dict):
        raise ValueError(
            f"{path.name}: top-level YAML must be a mapping, got {type(parsed).__name__}"
        )

    marker = parsed.pop(_PROVISIONAL_KEY, False)
    _warn_provisional(parsed, marker, path.name)

    if model is not None:
        return model.model_validate(parsed)
    return parsed
