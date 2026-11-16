"""Compare same-side, heel-strike-normalized joint moment waveforms."""

import argparse
import csv
from pathlib import Path

import numpy as np

from compare_trials import validate_pair
from run_inverse_dynamics import read_series


def read_strikes(path, side):
    """Read chronological heel strikes for the requested side."""
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"side", "time_s"}.issubset(reader.fieldnames or ()):
            raise ValueError(f"{path}: expected side and time_s columns.")
        strikes = []
        for row in reader:
            row_side = row["side"].strip().lower()
            if row_side not in ("left", "right"):
                raise ValueError(f"{path}: sides must be left or right.")
            if row_side == side:
                strikes.append(float(row["time_s"]))
    result = np.asarray(strikes)
    if result.size < 2 or not np.all(np.isfinite(result)) or np.any(np.diff(result) <= 0):
        raise ValueError(f"{path}: supply at least two finite, increasing {side} heel strikes.")
    return result


def normalize_trial(folder, events, side, columns, phase):
    table, time = read_series(folder / "inverse_dynamics.sto", 2)
    strikes = read_strikes(events, side)
    if strikes[0] < time[0] - 1e-8 or strikes[-1] > time[-1] + 1e-8:
        raise ValueError(f"{events}: heel strikes must be within the analysis output.")
    starts, ends = strikes[:-1], strikes[1:]
    if np.any(np.searchsorted(time, ends, side="right") -
              np.searchsorted(time, starts, side="left") < 2):
        raise ValueError(f"{events}: each stride needs at least two original motion samples.")
    target = starts[:, None] + (ends - starts)[:, None] * phase[None, :]
    missing = set(columns).difference(table.getColumnLabels())
    if missing:
        raise ValueError(f"{folder}: output columns not found: {', '.join(sorted(missing))}.")
    signals = [
        np.interp(target, time, table.getDependentColumn(column).to_numpy())
        for column in columns
    ]
    return np.stack(signals), ends - starts


def compare_gait(locked, active, locked_events, active_events, side, columns, output):
    """Write group means, stride variability and active-minus-locked waveforms."""
    locked, active, locked_events, active_events, output = map(
        Path, (locked, active, locked_events, active_events, output)
    )
    if side not in ("left", "right"):
        raise ValueError("Side must be left or right.")
    if not columns or len(set(columns)) != len(columns):
        raise ValueError("Supply distinct inverse-dynamics output column names.")
    validate_pair(locked, active)
    phase = np.linspace(0, 1, 101)
    baseline, baseline_times = normalize_trial(locked, locked_events, side, columns, phase)
    current, current_times = normalize_trial(active, active_events, side, columns, phase)

    baseline_mean, current_mean = baseline.mean(axis=1), current.mean(axis=1)
    baseline_sd = (baseline.std(axis=1, ddof=1) if baseline_times.size > 1
                   else np.full_like(baseline_mean, np.nan))
    current_sd = (current.std(axis=1, ddof=1) if current_times.size > 1
                  else np.full_like(current_mean, np.nan))
    with output.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("side", "column", "gait_percent", "locked_mean", "active_mean",
                         "delta_mean", "locked_stride_sd", "active_stride_sd",
                         "locked_strides", "active_strides",
                         "locked_mean_stride_s", "active_mean_stride_s"))
        for i, column in enumerate(columns):
            for frame in range(phase.size):
                before, after = baseline_mean[i, frame], current_mean[i, frame]
                writer.writerow((
                    side, column, frame, before, after, after - before,
                    "" if baseline_times.size == 1 else baseline_sd[i, frame],
                    "" if current_times.size == 1 else current_sd[i, frame],
                    baseline_times.size, current_times.size,
                    baseline_times.mean(), current_times.mean(),
                ))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--locked", required=True)
    parser.add_argument("--active", required=True)
    parser.add_argument("--locked-events", required=True)
    parser.add_argument("--active-events", required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--columns", nargs="+", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    try:
        output = compare_gait(args.locked, args.active, args.locked_events, args.active_events,
                              args.side, args.columns, args.out)
    except (AttributeError, KeyError, TypeError, ValueError, OSError, RuntimeError) as error:
        parser.exit(1, f"ATLAS: {error}\n")
    print(output)


if __name__ == "__main__":
    main()
