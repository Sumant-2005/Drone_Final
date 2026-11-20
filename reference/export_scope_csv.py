"""Export the 300 kg Python traces using the units shown on the Simulink scopes."""
from io import TextIOWrapper
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np

from emrax import motor_replay
from flight import Flight

ROOT = Path(__file__).resolve().parents[1]
BASE_COLUMNS = [
    'flight_time_s', 'forward_command_mps', 'right_command_mps', 'climb_command_mps',
    'yaw_rate_command_deg_s', 'height_target_m', 'forward_speed_mps', 'right_speed_mps',
    'climb_rate_mps', 'height_m', 'north_m', 'east_m', 'roll_deg', 'pitch_deg', 'yaw_deg',
    'collective_N', 'roll_moment_Nm', 'pitch_moment_Nm', 'yaw_moment_Nm',
]
MOTORS = ['M1_front_right', 'M2_back_left', 'M3_front_left', 'M4_back_right']
FLIGHT_COLUMNS = BASE_COLUMNS + [
    f'{motor}_{quantity}' for quantity in ['thrust_ref_N','speed_ref_rpm','shaft_rpm','load_Nm']
    for motor in MOTORS
]
DRIVE_COLUMNS = ['drive_time_s','mission_time_s','speed_command_rpm','shaft_rpm','load_Nm',
                 'electromagnetic_Nm','iq_command_peak_A','iq_peak_A','id_peak_A']


def scope_values(flight, scenario, history):
    t = history[:,0]
    profile = np.asarray(flight.p[scenario])
    commands = profile[np.searchsorted(profile[:,0], t, side='right')-1,1:]
    target = flight.p['initial_height_m'] + np.r_[0, np.cumsum(commands[:-1,2])*flight.dt]
    north_speed, east_speed = history[:,4], history[:,5]
    yaw = history[:,9]
    forward = np.cos(yaw)*north_speed + np.sin(yaw)*east_speed
    right = -np.sin(yaw)*north_speed + np.cos(yaw)*east_speed
    return np.column_stack((t, commands, target, forward, right, -history[:,6], -history[:,3],
                            history[:,1:3], np.rad2deg(history[:,7:10]), history[:,17:25],
                            history[:,25:29]*30/np.pi, history[:,13:17]*30/np.pi, history[:,29:33]))


def write_csv(bundle, name, columns, values):
    with bundle.open(name, 'w') as binary, TextIOWrapper(binary, encoding='ascii') as stream:
        np.savetxt(stream,values,fmt='%.11g',delimiter=',',header=','.join(columns),comments='')


def export(destination):
    flight = Flight()
    drive = json.loads((ROOT/'emrax_design.json').read_text())
    with ZipFile(destination, 'w', compression=ZIP_DEFLATED) as bundle:
        for scenario in ('demonstration', 'mission'):
            history = flight.run(scenario)
            traces = scope_values(flight,scenario,history)
            write_csv(bundle,f'flight_{scenario}.csv',FLIGHT_COLUMNS,traces)
            if scenario == 'mission':
                values = motor_replay(flight,history,drive)
                offset = drive['ramp_seconds'] + drive['settle_seconds']
                write_csv(bundle,'emrax_average_voltage.csv',DRIVE_COLUMNS,np.c_[values[:,0]+offset,values])


if __name__ == '__main__':
    export(ROOT/'report_figures/Python_scope_data_300kg.zip')
