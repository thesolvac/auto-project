"""Bootstrap smoke test (Phase 0).

Confirms the test harness runs and the top-level package imports cleanly.
Replaced/augmented by real tests as each layer is built.
"""

import autoproject


def test_package_imports() -> None:
    """The autoproject package must import without side effects."""
    assert autoproject.__name__ == "autoproject"
