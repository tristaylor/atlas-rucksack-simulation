"""Run and check a synthetic locked/active pair through OpenSim."""

import argparse
import csv
import math
from pathlib import Path
from xml.etree import ElementTree as xml

import opensim as osim
from numpy.testing import assert_allclose

from compare_gait_cycles import compare_gait
from compare_trials import compare_trials
from run_inverse_dynamics import LOADS, run_trial


BODIES = ("calcn_l", "calcn_r", "torso", "torso", "pelvis")


def make_model(path):
    """Create a small articulated fixture with known force moment arms."""
    model = osim.Model()
    model.setName("ATLAS_synthetic_fixture")
    model.setGravity(osim.Vec3(0, -9.80665, 0))
    pelvis = osim.Body("pelvis", 10, osim.Vec3(0), osim.Inertia(1))
    base = osim.SliderJoint("base", model.getGround(), osim.Vec3(0),
                            osim.Vec3(0, 0, math.pi / 2),
                            pelvis, osim.Vec3(0), osim.Vec3(0, 0, math.pi / 2))
    base.updCoordinate().setName("pelvis_ty")
    model.addBody(pelvis)
    model.addJoint(base)
    for body_name, coordinate, pin_x in (
        ("calcn_l", "knee_angle_l", -0.2),
        ("calcn_r", "knee_angle_r", 0.2),
        ("torso", "lumbar_extension", 0),
    ):
        body = osim.Body(body_name, 10, osim.Vec3(0), osim.Inertia(1))
        joint = osim.PinJoint(coordinate + "_joint", pelvis, osim.Vec3(pin_x, 0, 0),
                              osim.Vec3(0), body, osim.Vec3(0), osim.Vec3(0))
        joint.updCoordinate().setName(coordinate)
        model.addBody(body)
        model.addJoint(joint)
    # A known model force exercises the runner's actuator exclusion.
    actuator = osim.CoordinateActuator("lumbar_extension")
    actuator.setName("fixture_actuator")
    model.addForce(actuator)
    model.finalizeConnections()
    model.printToXML(str(path))


def write_motion(path, labels, time, samples):
    with path.open("w", encoding="ascii") as stream:
        stream.write(
            f"ATLAS synthetic fixture\nversion=1\nnRows={len(time)}\n"
            f"nColumns={len(labels) + 1}\ninDegrees=no\nendheader\n"
        )
        stream.write("\t".join(("time", *labels)) + "\n")
        for instant in time:
            stream.write("\t".join(f"{value:.17g}" for value in (instant, *samples)) + "\n")


def make_loads(folder, time, forces):
    folder.mkdir()
    root = xml.Element("OpenSimDocument", Version="40000")
    external = xml.SubElement(root, "ExternalLoads", name="externalloads")
    objects = xml.SubElement(external, "objects")
    xml.SubElement(external, "groups")
    xml.SubElement(external, "datafile").text = "trial_loads.mot"
    labels, samples = [], []
    for name, body, force, point_x in zip(LOADS, BODIES, forces, (-0.1, 0.3, 0.2, 0.2, 0.1)):
        entry = xml.SubElement(objects, "ExternalForce", name=name)
        properties = {
            "applied_to_body": body, "force_expressed_in_body": "ground",
            "point_expressed_in_body": "ground",
            "force_identifier": f"{name}_force_", "point_identifier": f"{name}_point_",
        }
        for key, value in properties.items():
            xml.SubElement(entry, key).text = value
        labels.extend(f"{name}_{kind}_{axis}" for kind in ("force", "point") for axis in "xyz")
        samples.extend((0, force, 0, point_x, 0, 0))
    xml.ElementTree(root).write(folder / "ExternalLoads.xml", encoding="utf-8", xml_declaration=True)
    write_motion(folder / "trial_loads.mot", labels, time, samples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path, help="New folder for generated demo files")
    args = parser.parse_args()
    output = args.out
    output.mkdir(parents=True, exist_ok=False)
    model = output / "fixture.osim"
    motion = output / "motion.mot"
    time = [index / 100 for index in range(101)]
    make_model(model)
    write_motion(motion, ("pelvis_ty", "knee_angle_l", "knee_angle_r", "lumbar_extension"),
                 time, [0] * 4)

    support = (40 * 9.80665 + 100) / 2
    results = {}
    for mode, forces in {
        "locked": (support, support, -30, -30, -40),
        "active": (support + 20, support - 20, -20, -20, -60),
    }.items():
        input_folder = output / f"{mode}_inputs"
        make_loads(input_folder, time, forces)
        result = run_trial(model, motion, input_folder / "ExternalLoads.xml", output / mode,
                           window=(0.2, 0.8), cutoff_hz=6)
        results[mode] = osim.TimeSeriesTable(str(result))
        assert_allclose(results[mode].getDependentColumn("pelvis_ty_force").to_numpy(),
                        0, atol=1e-8)

    # The moment changes are the negatives of the external moment changes.
    for label, expected in (
        ("lumbar_extension_moment", -4),
        ("knee_angle_l_moment", -2),
        ("knee_angle_r_moment", 2),
    ):
        active = results["active"].getDependentColumn(label).to_numpy()
        locked = results["locked"].getDependentColumn(label).to_numpy()
        assert_allclose(active - locked, expected, atol=1e-8)
    report = compare_trials(output / "locked", output / "active", output / "comparison.csv")
    locked_events = output / "locked_heel_strikes.csv"
    active_events = output / "active_heel_strikes.csv"
    locked_events.write_text("side,time_s\nleft,0.2\nleft,0.45\nleft,0.7\n", encoding="ascii")
    active_events.write_text("side,time_s\nleft,0.2\nleft,0.8\n", encoding="ascii")
    gait = compare_gait(output / "locked", output / "active", locked_events, active_events,
                        "left", ["knee_angle_l_moment", "lumbar_extension_moment"],
                        output / "gait_comparison.csv")
    with gait.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 202
    for row in rows:
        expected = -2 if row["column"] == "knee_angle_l_moment" else -4
        assert_allclose(float(row["delta_mean"]), expected, atol=1e-8)
        assert row["locked_strides"] == "2" and row["active_strides"] == "1"
        assert row["active_stride_sd"] == ""
    print(f"Analytical and gait comparison checks passed. Summary written to {report}.")


if __name__ == "__main__":
    main()
