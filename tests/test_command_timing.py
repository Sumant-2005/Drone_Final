"""Exercise zero-order-hold lookup on both sides of a rounded solver tick."""
import json
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_held_source_timing():
    design = json.loads((ROOT / 'design.json').read_text())
    drive = json.loads((ROOT / 'emrax_design.json').read_text())
    profile = np.array(design['mission'])
    dt = design['step_s']
    t = np.arange(round(profile[-1, 0] / dt) + 1) * dt
    commands = profile[np.searchsorted(profile[:, 0], t, side='right') - 1, 1:]
    height = design['initial_height_m'] + np.r_[0, np.cumsum(commands[:-1, 2])*dt]

    # A hit just before the stored time on the climb ramp returns stale height.
    k = round(29/dt)
    early = np.nextafter(t[k], -np.inf)
    old_index = np.searchsorted(t, early, side='right') - 1
    assert height[k] - height[old_index] == pytest.approx(0.00125)

    # Exercise every source sample, including command transitions, the drive
    # spin-up boundary and the final held value. The index is the exact oracle.
    warm = drive['ramp_seconds'] + drive['settle_seconds']
    drive_t = np.r_[np.arange(round(warm/dt))*dt, t+warm]
    for nominal in (t, drive_t):
        index = np.arange(nominal.size)
        source = np.r_[nominal[0], nominal[:-1] + np.diff(nominal)/2]
        for direction in (-np.inf, np.inf):
            hits = np.maximum(nominal[0], np.nextafter(nominal, direction))
            np.testing.assert_array_equal(np.searchsorted(source, hits, side='right')-1, index)
    print(f'One-tick height error reproduced: {height[k]-height[old_index]:.5f} m. '
          f'Corrected held lookup: {len(t)} flight ticks and {len(drive_t)} drive ticks passed.')
