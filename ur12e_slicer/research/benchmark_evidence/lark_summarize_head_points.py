"""Reproduce head-phantom descriptive point tables from the released CSV.

Usage: python Tracking/summarize_head_points.py --datasets LARK_Datasets.zip \
           --output /path/to/head-point-tables
"""

import argparse
import hashlib
import io
import json
from pathlib import Path
import zipfile

import numpy as np
import pandas as pd


FILENAME = "Dataset 2 (Head Phantom).csv"
KEYS = ["Trial", "Config", "Tracking", "Fusion", "Cams", "Metric"]
CONFIGS = ["top", "right", "left"]
FUSIONS = ["Basic", "Kalman", "Kalman Adaptive"]
# Match the paper's occlusion diagram by active-camera subsets. For example,
# CSV 'left' keeps cameras [1, 2, 3] at three cameras: paper right occlusion.
OCCLUSION = {"pooled": "pooled", "top": "top", "left": "right", "right": "left"}


def load_points(source):
    if source.is_dir():
        raw = (source / FILENAME).read_bytes()
    else:
        with zipfile.ZipFile(source) as archive:
            raw = archive.read("LARK_Datasets/" + FILENAME)
    frame = pd.read_csv(io.BytesIO(raw))
    points = frame.loc[(frame.Device == "Ring") & frame.Metric.isin(["FRE", "TRE"]),
                       KEYS + ["Point", "Error"]].copy()
    expected = []
    for trial in range(1, 21):
        for config in CONFIGS:
            for tracking, first in [("Mono", 1), ("Stereo", 2)]:
                for fusion in FUSIONS:
                    for cams in range(first, 6):
                        for metric, labels in [("FRE", [f"R{i}" for i in range(1, 10)]),
                                               ("TRE", [f"{i:02d}" for i in range(1, 11)])]:
                            expected.extend((trial, config, tracking, fusion, cams, metric, label)
                                            for label in labels)
    index = pd.MultiIndex.from_frame(points[KEYS + ["Point"]])
    expected = pd.MultiIndex.from_tuples(expected, names=index.names)
    if index.has_duplicates or len(index) != len(expected) or len(expected.difference(index)):
        raise ValueError("Missing, duplicate, or unexpected head point measurements")
    if not np.isfinite(points.Error).all() or (points.Error < 0).any():
        raise ValueError("Point errors must be finite and nonnegative")
    five_keys = [key for key in KEYS if key != "Config"] + ["Point"]
    if not points.loc[points.Cams == 5].groupby(five_keys).Error.nunique().eq(1).all():
        raise ValueError("Five-camera scenario copies differ")
    return points, hashlib.sha256(raw).hexdigest()


def summarize(points):
    trial = points.assign(SquaredError=points.Error ** 2).groupby(KEYS).SquaredError.mean()
    trial = np.sqrt(trial).rename("RMSE").reset_index()
    # Preserve the historical descriptive tables' four-decimal intermediate
    # precision and population standard deviation (ddof=0).
    trial["TableRMSE"] = trial.RMSE.map(lambda value: float(f"{value:.4f}"))
    records = []
    for scenario in ["pooled"] + CONFIGS:
        subset = trial if scenario == "pooled" else trial.loc[trial.Config == scenario]
        for (metric, cams, tracking, fusion), group in subset.groupby(
                ["Metric", "Cams", "Tracking", "Fusion"]):
            values = group.TableRMSE.to_numpy()
            median = float(np.median(values))
            deviation = float(np.std(values, ddof=0))
            records.append(dict(scenario=OCCLUSION[scenario], csv_config=scenario,
                                metric=metric, cams=int(cams),
                                tracking=tracking, fusion=fusion, n=len(values),
                                median=median, standard_deviation=deviation,
                                latex=f"{median:.4f} (±{deviation:.4f})"))
    return trial, pd.DataFrame(records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    points, digest = load_points(args.datasets)
    trial, tables = summarize(points)
    args.output.mkdir(parents=True, exist_ok=True)
    trial.to_csv(args.output / "head-point-trial-rmse.csv", index=False)
    tables.to_csv(args.output / "head-point-table-cells.csv", index=False)
    (args.output / "head-point-provenance.json").write_text(json.dumps({
        "input_file": FILENAME, "input_sha256": digest,
        "trial_rmse": "sqrt(mean(squared point errors)) within trial and configuration",
        "table_aggregation": "median and population standard deviation of trial/configuration RMSEs",
        "intermediate_precision": "four decimals, matching the historical JSON summary precision",
        "pooled_configurations": CONFIGS,
        "csv_config_to_paper_occlusion": OCCLUSION,
        "trajectory_tables": "not generated: the CSV does not contain trajectory sample counts",
    }, indent=2) + "\n")
    print(f"Wrote {len(trial)} trial/configuration RMSEs and {len(tables)} table cells")


if __name__ == "__main__":
    main()
