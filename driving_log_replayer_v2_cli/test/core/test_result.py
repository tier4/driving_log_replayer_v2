from pathlib import Path

from driving_log_replayer_v2_cli.core.result import display_all


def test_display_all() -> None:
    display_all(Path.home().joinpath("dlr2_data"), "latest")
