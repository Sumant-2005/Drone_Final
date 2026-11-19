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
            assert not tree.findtext(".//P[@Name='PreLoadFcn']")
            assert tree.findtext(".//P[@Name='PostLoadFcn']") == (
                "addpath(fileparts(get_param('EMRAXDrive','FileName')),'-begin'); MotorParameters(false);")
            assert "addpath(fileparts(get_param('DroneFlight','FileName')),'-begin'); FlightParameters;" in model
            assert tree.findtext(".//P[@Name='InitFcn']")=='MotorParameters;'
    print('4 self-contained files; flight output, motor input, mission and drive callbacks checked.')


def test_serialized_flight_ports_and_branches():
    # Inspect the delivered MDL text, independently of the diagram generator.
    from collections import defaultdict
    with ZipFile(ROOT/'downloads/Drone_BTP_Four_Files.zip') as bundle:
        mdl = bundle.read('Drone_BTP/DroneFlight.mdl').decode()
    stack = []
    sections = []
    for raw in mdl.splitlines():
        text = raw.strip()
        if text.endswith(' {'):
            node = defaultdict(list)
            if stack:
                stack[-1][text[:-2]].append(node)
            stack.append(node)
            if text == 'System {':
                sections.append(node)
        elif text == '}':
            stack.pop()
        else:
            key,value = text.split(' ',1)
            stack[-1][key].append(value.strip('"'))
    assert not stack
    total = 0
    for section in sections:
        blocks = {b['Name'][0]: b for b in section['Block']}
        ports = {name: tuple(json.loads(b['Ports'][0])) for name,b in blocks.items()}
        used = set()
        for line in section['Line']:
            source = line['SrcBlock'][0]
            assert 1 <= int(line['SrcPort'][0]) <= ports[source][1]
            if line['Branch']:
                assert line['Points'] and line['Points'][0] != '[0, 0]'
            for endpoint in line['Branch'] or [line]:
                dest = endpoint['DstBlock'][0]
                port = int(endpoint['DstPort'][0])
                assert 1 <= port <= ports[dest][0]
                assert (dest,port) not in used
                used.add((dest,port))
        for name,(nin,nout) in ports.items():
            assert all((name,p) in used for p in range(1,nin+1))
            if blocks[name]['System']:
                kids = blocks[name]['System'][0]['Block']
                assert (nin,nout) == tuple(sum(k['BlockType']==[t] for k in kids)
                                           for t in ('Inport','Outport'))
        if 'Upward force' in blocks:
            assert ports['Upward force'] == (3,1) and ('Upward force',3) in used
        total += len(used)
    print(f'Checked {total} destinations in {len(sections)} serialized flight diagrams.')
