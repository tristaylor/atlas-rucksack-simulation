"""Run OpenSim inverse dynamics on an ATLAS trial exported from MATLAB."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import opensim as osim


LOADS = ("left_grf", "right_grf", "left_shoulder", "right_shoulder", "hip_belt")


def read_series(path, minimum_rows):
    """Read and validate an OpenSim scalar time series."""
    table = osim.TimeSeriesTable(str(path))
    time = tuple(table.getIndependentColumn())
    labels = tuple(table.getColumnLabels())
    if len(time) < minimum_rows or len(set(labels)) != len(labels):
        raise ValueError(f"{path}: expected at least {minimum_rows} rows and unique labels.")
    if not all(map(math.isfinite, time)) or any(b <= a for a, b in zip(time, time[1:])):
        raise ValueError(f"{path}: time must be finite and strictly increasing.")
    for label in labels:
        if not np.all(np.isfinite(table.getDependentColumn(label).to_numpy())):
            raise ValueError(f"{path}: non-finite samples in {label}.")
    return table, time


def validate_loads(path, model):
    root = ElementTree.parse(path).getroot()
    external = root.find("ExternalLoads")
    if external is None:
        raise ValueError(f"{path}: missing ExternalLoads element.")
    filename = external.findtext("datafile", "").strip()
    if not filename:
        raise ValueError(f"{path}: missing load table path.")
    load_file = (path.parent / filename).resolve()
    table, time = read_series(load_file, 2)
    step = (time[-1] - time[0]) / (len(time) - 1)
    if any(abs(b - a - step) > 1e-6 * step for a, b in zip(time, time[1:])):
        raise ValueError(f"{load_file}: the load table must use a uniform time grid.")

    group_element = external.find("objects")
    objects = list(group_element) if group_element is not None else []
    if sorted(entry.get("name", "") for entry in objects) != sorted(LOADS):
        raise ValueError(f"{path}: expected exactly one mapping for each ATLAS load group.")
    bodies = model.getBodySet()
    body_names = {bodies.get(i).getName() for i in range(bodies.getSize())}
    labels = tuple(table.getColumnLabels())
    for entry in objects:
        prefix = entry.get("name")
        body = entry.findtext("applied_to_body", "").strip()
        if entry.tag != "ExternalForce" or body not in body_names:
            raise ValueError(f"{prefix}: invalid target body {body!r}; use a model body name.")
        for frame in ("force_expressed_in_body", "point_expressed_in_body"):
            if entry.findtext(frame, "").strip() != "ground":
                raise ValueError(f"{prefix}: {frame} must be 'ground'.")
        for kind in ("force", "point", "torque"):
            identifier = entry.findtext(f"{kind}_identifier", "").strip()
            expected = f"{prefix}_{kind}_"
            group = tuple(label for label in labels if label.startswith(expected))
            if kind == "torque" and identifier in ("", "Unassigned") and not group:
                continue
            if identifier != expected or group != tuple(expected + axis for axis in "xyz"):
                raise ValueError(f"{prefix}: expected ordered columns {expected}[x,y,z].")
    return load_file, time


def write_summary(path, table, root_columns):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("column", "category", "min", "max", "rms", "signed_peak"))
        for label in table.getColumnLabels():
            values = table.getDependentColumn(label).to_numpy()
            low, high = float(min(values)), float(max(values))
            rms = float(np.linalg.norm(values) / math.sqrt(len(values)))
            category = "ground_joint_residual" if label in root_columns else "joint_coordinate"
            writer.writerow((label, category, low, high, rms, max((low, high), key=abs)))


def run_trial(model_path, motion_path, loads_path, output, *, window=None, cutoff_hz=-1):
    """Validate the handoff, run OpenSim and return the output .sto path."""
    model_path, motion_path, loads_path, output = [
        Path(path).resolve() for path in (model_path, motion_path, loads_path, output)
    ]
    if not math.isfinite(cutoff_hz) or (cutoff_hz != -1 and cutoff_hz <= 0):
        raise ValueError("Cutoff must be -1 (disabled) or a positive frequency in Hz.")
    model = osim.Model(str(model_path))
    model.initSystem()
    motion, motion_time = read_series(motion_path, 6)
    if motion.getTableMetaDataAsString("inDegrees").lower() not in ("yes", "no"):
        raise ValueError("Coordinate motion must declare inDegrees=yes or inDegrees=no.")
    coordinates = model.getCoordinateSet()
    coordinate_names = {coordinates.get(i).getName() for i in range(coordinates.getSize())}
    missing = coordinate_names.difference(motion.getColumnLabels())
    missing.difference_update(
        coordinates.get(i).getName() for i in range(coordinates.getSize())
        if coordinates.get(i).getDefaultLocked()
    )
    if missing:
        raise ValueError(f"Motion is missing model coordinates: {', '.join(sorted(missing))}.")
    load_file, load_time = validate_loads(loads_path, model)
    if window is None:
        window = (motion_time[0], motion_time[-1])
    start, end = window
    if not max(motion_time[0], load_time[0]) <= start < end <= min(motion_time[-1], load_time[-1]):
        raise ValueError("Analysis window must be covered by both motion and loads; supply --window.")
    samples = [time for time in motion_time if start <= time <= end]
    if len(samples) < 2:
        raise ValueError("Analysis window must include at least two coordinate samples.")
    if cutoff_hz != -1:
        max_step = max(b - a for a, b in zip(motion_time, motion_time[1:]))
        if cutoff_hz >= 0.5 / max_step:
            raise ValueError("Cutoff must be below the motion sample Nyquist frequency.")

    forces = model.getForceSet()
    exclusions = set()
    for index in range(forces.getSize()):
        force = forces.get(index)
        if osim.ExternalForce.safeDownCast(force) is not None:
            raise ValueError("Use a wearer model without pre-attached external loads.")
        if not force.get_appliesForce():
            continue
        if osim.Actuator.safeDownCast(force) is not None:
            exclusions.add("actuators")
        elif forces.getNumGroups() == 0:
            if force.getName().lower() in {"all", "actuators", "muscles"}:
                raise ValueError(f"Rename model force {force.getName()!r}; it shadows a tool keyword.")
            exclusions.add(force.getName())
        else:
            # OpenSim searches only groups for individual names once groups exist.
            memberships = osim.ArrayStr()
            forces.getGroupNamesContaining(force.getName(), memberships)
            groups = {memberships.get(i) for i in range(memberships.getSize())}
            usable = sorted(group for group in groups
                            if group.lower() not in {"all", "actuators", "muscles"})
            if not usable:
                raise ValueError(f"Disable or group model force {force.getName()!r} before analysis.")
            exclusions.add(usable[0])
    excluded = osim.ArrayStr()
    for name in sorted(exclusions):
        excluded.append(name)

    ground_coordinates = set()
    joints = model.getJointSet()
    for index in range(joints.getSize()):
        joint = joints.get(index)
        if joint.getParentFrame().findBaseFrame().getName() == "ground":
            for coordinate in range(joint.numCoordinates()):
                ground_coordinates.add(joint.get_coordinates(coordinate).getName())

    # Each trial gets its own output folder. Refuse to overwrite earlier results.
    output.mkdir(parents=True, exist_ok=False)
    output_file = output / "inverse_dynamics.sto"
    tool = osim.InverseDynamicsTool()
    tool.setName("ATLAS")
    tool.setModelFileName(str(model_path))
    tool.setCoordinatesFileName(str(motion_path))
    tool.setExternalLoadsFileName(str(loads_path))
    tool.setExcludedForces(excluded)
    tool.setLowpassCutoffFrequency(cutoff_hz)
    tool.setStartTime(samples[0])
    tool.setEndTime(samples[-1])
    tool.setResultsDir(str(output))
    tool.setOutputGenForceFileName(output_file.name)
    tool.printToXML(str(output / "ID_Setup.xml"))
    if not tool.run():
        raise RuntimeError("OpenSim inverse dynamics did not complete.")

    results, result_time = read_series(output_file, 2)
    if cutoff_hz == -1:
        if len(result_time) != len(samples) or not np.allclose(
            result_time, samples, rtol=0, atol=1e-8
        ):
            raise RuntimeError("OpenSim output times differ from the selected motion samples.")
    else:
        # OpenSim pads and can resample motion during filtering, then selects
        # the nearest preceding sample at each endpoint on the resulting grid.
        max_step = max(b - a for a, b in zip(motion_time, motion_time[1:]))
        precision = 1e-5 * max(1, motion_time[-1] - motion_time[0])
        if (abs(result_time[0] - samples[0]) > max_step + precision
                or abs(result_time[-1] - samples[-1]) > max_step + precision
                or result_time[0] < load_time[0] - precision
                or result_time[-1] > load_time[-1] + precision):
            raise RuntimeError("Filtered output falls outside the expected load or motion window.")
    root_columns = {
        name + suffix for name in ground_coordinates for suffix in ("_force", "_moment")
    }
    write_summary(output / "summary.csv", results, root_columns)
    manifest = {
        "opensim_version": osim.GetVersion(),
        "model": str(model_path),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "motion": str(motion_path),
        "external_loads": str(loads_path),
        "load_table": str(load_file),
        "requested_window_s": [start, end],
        "output_window_s": [result_time[0], result_time[-1]],
        "motion_lowpass_hz": cutoff_hz,
        "excluded_model_forces": [excluded.get(i) for i in range(excluded.getSize())],
    }
    (output / "analysis.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return output_file


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Participant-scaled .osim model")
    parser.add_argument("--motion", required=True, help="OpenSim coordinate motion from IK")
    parser.add_argument("--loads", required=True, help="MATLAB-exported ExternalLoads.xml")
    parser.add_argument("--out", required=True, help="New output directory")
    parser.add_argument("--window", type=float, nargs=2, metavar=("START", "END"),
                        help="Analysis interval in seconds; defaults to the full motion")
    parser.add_argument("--cutoff-hz", type=float, default=-1,
                        help="Low-pass filter for coordinate motion only (-1 disables)")
    args = parser.parse_args()
    try:
        output = run_trial(args.model, args.motion, args.loads, args.out,
                           window=args.window, cutoff_hz=args.cutoff_hz)
    except (ValueError, OSError, RuntimeError, ElementTree.ParseError) as error:
        parser.exit(1, f"ATLAS: {error}\n")
    print(output)


if __name__ == "__main__":
    main()
