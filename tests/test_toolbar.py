import io
import json
import re
from pathlib import Path
from xml.etree import ElementTree as ET
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def test_toolbar_bundle():
    names = {'DroneFlight.mdl','EMRAXDrive.slx','FlightParameters.m','MotorParameters.m'}
    with ZipFile(ROOT/'downloads/Drone_BTP_Four_Files.zip') as bundle:
        assert bundle.testzip() is None
        assert set(bundle.namelist()) == {'Drone_BTP/'+n for n in names}
        for n in names:
            assert bundle.read('Drone_BTP/'+n) == (ROOT/'toolbar'/n).read_bytes()
        model = bundle.read('Drone_BTP/DroneFlight.mdl').decode()
        flight = bundle.read('Drone_BTP/FlightParameters.m').decode()
        motor = bundle.read('Drone_BTP/MotorParameters.m').decode()
        assert 'SaveFormat "StructureWithTime"' in model and 'ReturnWorkspaceOutputs off' in model
        assert 'OutputSaveName "motor1_flight_replay"' in model and 'InitFcn "FlightParameters;"' in model
        assert "evalin('base','motor1_flight_replay')" in motor
        assert 'setup_flight' not in model and '.json' not in flight+motor and '.mat' not in flight+motor
        profile = re.search(r'p\.mission = \[(.*?)\];',flight,re.S).group(1)
        rows = [list(map(float,line.split())) for line in profile.strip().splitlines()]
        np.testing.assert_array_equal(rows,json.loads((ROOT/'design.json').read_text())['mission'])
        with ZipFile(io.BytesIO(bundle.read('Drone_BTP/EMRAXDrive.slx'))) as drive, ZipFile(ROOT/'EMRAXMission.slx') as original:
            assert drive.testzip() is None and set(drive.namelist()) == set(original.namelist())
            diff=[n for n in drive.namelist() if drive.read(n)!=original.read(n)]
            assert diff==['simulink/blockdiagram.xml']
            tree=ET.fromstring(drive.read(diff[0]))
            assert tree.findtext(".//P[@Name='PreLoadFcn']")=='MotorParameters(false);'
            assert tree.findtext(".//P[@Name='InitFcn']")=='MotorParameters;'
    print('4 self-contained files; flight output, motor input, mission and drive callbacks checked.')
