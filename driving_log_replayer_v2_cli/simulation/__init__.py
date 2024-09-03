from pathlib import Path

import click

from driving_log_replayer_v2_cli.core.result import display_all
from driving_log_replayer_v2_cli.simulation.run import run as sim_run

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


@click.group(context_settings=CONTEXT_SETTINGS)
def simulation() -> None:
    """Run simulation and check simulation log."""


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument(
    "scenario_root_directory",
    type=click.Path(exists=True, file_okay=False, resolve_path=True, path_type=Path),
)
@click.option("--launch_args", "-l", multiple=True, default=[])
def run(scenario_root_directory: Path, launch_args: list[str]) -> None:
    sim_run(scenario_root_directory, launch_args)


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument(
    "log_root",
    type=click.Path(exists=True, file_okay=False, resolve_path=True, path_type=Path),
)
@click.option("--target_dir_name", "-t", type=str, default="**")
def show_result(log_root: Path, target_dir_name: str) -> None:
    """Show summary of simulation results in log_root."""
    display_all(log_root, target_dir_name)
