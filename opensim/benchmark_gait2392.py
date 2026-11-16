"""Benchmark the ATLAS handoff against OpenSim's public Gait2392 trial."""

import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import opensim as osim

from run_inverse_dynamics import LOADS, read_series, run_trial


def benchmark(data, output):
    """Run identical model and motion with the two external-load formats."""
    data, output = Path(data).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    model = data / 'OutputReference' / 'subject01_simbody.osim'
    motion = data / 'subject01_walk1_ik.mot'
    table = osim.TimeSeriesTable(str(data / 'subject01_walk1_grf.mot'))
    original_t = tuple(table.getIndependentColumn())
    # The published 600 Hz clock was rounded to four decimal places. Restore
    # the uniform sample times while preserving each recorded sample value.
    time = np.linspace(original_t[0], original_t[-1], len(original_t))
    source = ('1_ground_force_v', 'ground_force_v')
    source_point = ('1_ground_force_p', 'ground_force_p')
    source_torque = ('1_ground_torque_', 'ground_torque_')

    document = ET.Element('OpenSimDocument', Version='40000')
    ext = ET.SubElement(document, 'ExternalLoads', name='externalloads')
    objs = ET.SubElement(ext, 'objects')
    ET.SubElement(ext, 'groups')
    ET.SubElement(ext, 'datafile').text = 'trial_loads.mot'
    labels = ['time']
    signals = [time]
    for i, (prefix, body) in enumerate(zip(LOADS, ('calcn_l', 'calcn_r','torso','torso','pelvis'))):
        element = ET.SubElement(objs, 'ExternalForce', name=prefix)
        ET.SubElement(element, 'applied_to_body').text = body
        for field in ('force_expressed_in_body','point_expressed_in_body'):
            ET.SubElement(element, field).text = 'ground'
        for kind, sources in [('force', source), ('point', source_point), ('torque', source_torque)]:
            if i >= 2 and kind == 'torque':
                continue
            target = f'{prefix}_{kind}_'
            ET.SubElement(element, f'{kind}_identifier').text = target
            for axis in 'xyz':
                labels.append(target+axis)
                values = (table.getDependentColumn(sources[i]+axis).to_numpy()
                          if i < 2 else np.zeros_like(time))
                signals.append(values)

    ET.ElementTree(document).write(output / 'ExternalLoads.xml', encoding='utf-8')
    np.savetxt(output / 'trial_loads.mot', np.column_stack(signals), fmt='%.17g', delimiter='\t',
               header=f'ATLAS benchmark\nversion=1\nnRows={len(time)}\nnColumns={len(labels)}\ninDegrees=no\nendheader\n'+'\t'.join(labels),
               comments='')
    atlas = run_trial(model, motion, output / 'ExternalLoads.xml', output / 'atlas',
                      window=(0.5, 1.6), cutoff_hz=6)

    native_dir = output / 'native'
    native_dir.mkdir()
    native = osim.InverseDynamicsTool()
    native.setModelFileName(str(model))
    native.setCoordinatesFileName(str(motion))
    native.setExternalLoadsFileName(str(data / 'subject01_walk1_grf.xml'))
    native.setResultsDir(str(native_dir))
    native.setOutputGenForceFileName('native.sto')
    native.setLowpassCutoffFrequency(6)
    native.setStartTime(0.5)
    native.setEndTime(1.6)
    exclude = osim.ArrayStr()
    exclude.append('muscles')
    native.setExcludedForces(exclude)
    if not native.run():
        raise RuntimeError("The native OpenSim benchmark failed.")
    a, ta = read_series(atlas, 2)
    b, tb = read_series(native_dir / 'native.sto', 2)
    np.testing.assert_allclose(ta, tb, rtol=0, atol=1e-8)
    for label in ('knee_angle_l_moment','knee_angle_r_moment','lumbar_extension_moment'):
        error = a.getDependentColumn(label).to_numpy() - b.getDependentColumn(label).to_numpy()
        peak = np.max(np.abs(error))
        print(f'{label}: peak absolute difference {peak:.6f} Nm over {len(ta)} frames')
        if peak >= 0.1:
            raise RuntimeError(f"{label}: benchmark difference exceeds 0.1 Nm.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", required=True,
                        help="Path to the upstream Pipelines/Gait2392_Simbody directory")
    parser.add_argument("--out", required=True, help="New directory for benchmark outputs")
    args = parser.parse_args()
    benchmark(args.upstream, args.out)


if __name__ == "__main__":
    main()
