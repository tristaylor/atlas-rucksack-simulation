"""Compare generalized force and moment summaries for a matched trial pair."""

import argparse
import csv
import json
from pathlib import Path


def validate_pair(locked, active):
    """Require the same model and numerical settings before a comparison."""
    if locked.resolve() == active.resolve():
        raise ValueError("Supply two distinct analysis directories.")
    runs = [json.loads((folder / "analysis.json").read_text(encoding="utf-8"))
            for folder in (locked, active)]
    for field in ("model_sha256", "opensim_version", "motion_lowpass_hz", "excluded_model_forces"):
        if runs[0][field] != runs[1][field]:
            raise ValueError(f"Trial analyses differ in {field}.")


def compare_trials(locked, active, output):
    """Write active-minus-locked differences in extrema, signed peaks and RMS."""
    locked, active, output = map(Path, (locked, active, output))
    validate_pair(locked, active)

    summaries = []
    for folder in (locked, active):
        with (folder / "summary.csv").open(encoding="utf-8", newline="") as stream:
            summaries.append({row["column"]: row for row in csv.DictReader(stream)})
    if summaries[0].keys() != summaries[1].keys():
        raise ValueError("Trials have different output columns.")

    quantities = ("min", "max", "rms", "signed_peak")
    with output.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        labels = ["column", "category"]
        for quantity in quantities:
            labels.extend((f"locked_{quantity}", f"active_{quantity}", f"delta_{quantity}"))
        writer.writerow(labels)
        for column, baseline in summaries[0].items():
            current = summaries[1][column]
            row = [column, baseline["category"]]
            for quantity in quantities:
                before, after = float(baseline[quantity]), float(current[quantity])
                row.extend((before, after, after - before))
            writer.writerow(row)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--locked", required=True, help="Output directory for the locked trial")
    parser.add_argument("--active", required=True, help="Output directory for the active trial")
    parser.add_argument("--out", required=True, help="Path for a new comparison CSV")
    args = parser.parse_args()
    try:
        output = compare_trials(args.locked, args.active, args.out)
    except (KeyError, ValueError, OSError) as error:
        parser.exit(1, f"ATLAS: {error}\n")
    print(output)


if __name__ == "__main__":
    main()
