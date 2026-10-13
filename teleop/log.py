"""Run logs: the per-cycle CSV and a copy of the settings used, next to each other in the log folder."""
import csv
from datetime import datetime
from pathlib import Path

import yaml


def run_paths(log_dir, mode, root):
    """(csv path, settings copy path) for a new run, e.g. data/teleop/teleop_sim_20261012_130000.csv."""
    folder = Path(log_dir)
    if not folder.is_absolute():
        folder = Path(root) / folder
    folder.mkdir(parents=True, exist_ok=True)
    stem = f"teleop_{mode}_{datetime.now():%Y%m%d_%H%M%S}"
    return folder / f"{stem}.csv", folder / f"{stem}_settings.yaml"


def save_run(rows, header, raw_settings, csv_path, settings_path):
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    with open(settings_path, "w") as f:
        yaml.safe_dump(raw_settings, f, sort_keys=False)
