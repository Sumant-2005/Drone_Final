from pathlib import Path
import json
import sys
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'reference'))
from flight import Flight
from emrax import motor_replay


def test_final_drive_model():
    # Inspect the actual deliverable, including its referenced charts and connections.
    with ZipFile(ROOT/'EMRAXMission.slx') as z:
        assert z.testzip() is None
        def system(name):
            return ET.fromstring(z.read(f'simulink/systems/system_{name}.xml'))
        def block(s,sid):
            return s.find(f"Block[@SID='{sid}']")
        def param(b,k):
            return b.findtext(f"P[@Name='{k}']")
        def targets(s,sid):
            return {node.text for line in s.findall('Line') if param(line,'Src')==f'{sid}#out:1'
                    for node in line.findall(".//P[@Name='Dst']")}
        r,s,q=system('root'),system(40),system(14)
        assert param(block(r,135),'VariableName')=='shaft_load'
        assert targets(r,135)=={'132#in:1','192#in:1'}
        assert param(block(s,187),'VariableName')=='shaft_speed_command'
        assert targets(s,187)=={'188#in:1'}
        assert targets(s,50)=={'208#in:1'} and param(block(s,208),'Gain')=='pi/30'
        assert targets(s,208)=={'44#in:1','45#in:1'}
        assert param(block(q,17),'Value')=='emrax.flux' and param(block(q,20),'GotoTag')=='w_elec'
        assert sum('IGBTDiodeLib' in (param(b,'SourceBlock') or '') for b in r.findall('Block'))==6
        panel=system(152)
        assert {b.get('Name') for b in panel.findall("Block[@BlockType='Scope']")}=={
            'Current - A peak','Speed - rpm','Torque - Nm'}
        for sysfile in [n for n in z.namelist() if n.startswith('simulink/systems/system_')]:
            tree=ET.fromstring(z.read(sysfile))
            ids={b.get('SID') for b in tree.findall('Block')}
            for endpoint in tree.findall(".//Line//P"):
                if endpoint.get('Name') in ('Src','Dst'):
                    assert endpoint.text.split('#')[0] in ids


def test_mission_and_average_drive():
    f=Flight()
    p=json.loads((ROOT/'emrax_design.json').read_text())
    h=f.run('mission')
    assert f.mass==300
    hover=f.hover*30/np.pi
    peak=h[:,25:29].max()*30/np.pi
    assert hover==pytest.approx(4650,abs=1) and peak<p['speed_limit']
    a=motor_replay(f,h,p)
    assert np.isfinite(a).all()
    mission=a[:,0]>=0
    np.testing.assert_allclose(a[mission,1],h[:,25]*30/np.pi)
    np.testing.assert_allclose(a[mission,3],h[:,29])
    error=a[:,2]-a[:,1]
    rms=np.sqrt(np.mean(error[mission]**2))
    assert max(abs(error[(a[:,0]>=-.2)&(a[:,0]<0)]))<.01*hover
    assert rms<.01*hover and max(a[mission,2])<p['speed_limit']
    assert max(abs(a[:,5]))<=p['continuous_phase_arms']*np.sqrt(2)+1e-8
    assert max(abs(a[mission,4]))>max(a[mission,3])  # torque to accelerate the inertia
    print(f'300 kg flight: hover {hover:.1f} rpm; max demand {peak:.1f} rpm')
    print(f'M1 average-voltage drive: RMS speed error {rms:.2f} rpm; '
          f'peak absolute error {max(abs(error[mission])):.2f} rpm; max speed {max(a[mission,2]):.1f} rpm')
