"""Rebuild the four-file toolbar download from the project models."""
from xml.etree import ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED
from pathlib import Path
import json
from generate import graph, emit_mdl
def configure_scopes(panel):
    # R2023a Scope configuration applies to displayed history as well as logging.
    # Auto spans the complete finite run, including after a mission duration edit.
    return ' '.join([
        f"scopePaths = find_system('{panel}','BlockType','Scope');",
        "for scopeIndex = 1:numel(scopePaths),",
        "scopeCfg = get_param(scopePaths{scopeIndex},'ScopeConfiguration');",
        "scopeCfg.DataLoggingLimitDataPoints = false;",
        "scopeCfg.DataLoggingDecimateData = false;",
        "scopeCfg.TimeSpan = 'Auto';",
        "scopeCfg.TimeDisplayOffset = '0';",
        "scopeCfg.AxesScaling = 'Auto';",
        "end;",
    ])


g=graph()
g.add('Reference','powergui',25,250,130,60,
      SourceBlock='sps_lib/powergui',SourceType='PSB option menu block')
replay=g.add('SubSystem','Front right motor replay',760,250,185,60)
# Align the display tags with the output branches along the top-level chain.
placements = {
    'Flight command':[20,85,180,145], 'Flight controller':[320,100,530,160],
    'Motor mixer':[675,100,860,160], 'Thrust to speed':[1010,100,1200,160],
    'Drone dynamics':[1345,100,1530,160], 'Read flight state':[85,185,230,217],
    'Flight scopes':[385,250,555,310],
}
for b in g.blocks:
    if b[1] in placements:
        b[2][:] = placements[b[1]]
for source, tag in [('Flight command','Send flight command'),('Flight controller','Send thrust and moments'),
                    ('Motor mixer','Send motor thrust'),('Thrust to speed','Send motor speed reference'),
                    ('Drone dynamics','Send flight state')]:
    right = placements[source][2]
    next(b[2] for b in g.blocks if b[1]==tag)[:] = [right+42,20,right+210,50]
out = g.add('ToFile', 'Save motor 1 replay', 1060, 260, 180, 45,
            FileName='Motor1Replay.mat', MatrixName='frontRightMotor',
            SaveFormat='Array', Decimation='1', SampleTime='-1')
g.wire('Front right motor replay',out)
ref=replay.read('speedRef',22,50)
state=replay.read('state',22,158)
a=replay.matrix('Motor 1 speed demand',1,0,ref,'[1 0 0 0]')
rpm=replay.gain('Speed demand in rpm',2,0,a,'30/pi')
b=replay.matrix('Motor 1 shaft speed',1,1,state,'[zeros(1,12) 1 0 0 0]')
w2=replay.prod('Speed squared',2,1,b,b)
load=replay.gain('Resisting load in Nm',3,1,w2,'drone.kQ')
pair=replay.mux('Speed and load',4,0,[rpm,load])
hold=replay.op('ZeroOrderHold','Replay samples',5,0,[pair],SampleTime='drone.step_s')
replay.output('Replay',1,6,0,hold)
# The callback code lives in the models. The delivered .m files contain settings only.
callbacks = Path(__file__).resolve().parent/'toolbar_callbacks'
flight_setup = (callbacks/'flight_init.m').read_text()
drive_setup = (callbacks/'motor_settings.m').read_text()
drive_replay = (callbacks/'motor_replay.m').read_text()
# Use ordinary MATLAB statements in the MDL text token (no newline escaping).
flight_command = ' '.join(line.strip() for line in flight_setup.splitlines()
                          if line.strip() and not line.lstrip().startswith('%'))
folder = "fileparts(get_param('DroneFlight','FileName'))"
# FileName is a literal block dialog field. Resolve it using the loaded model path.
set_path = ("set_param('DroneFlight/Save motor 1 replay','FileName',"
            f"fullfile({folder},'Motor1Replay.mat'));")
s = emit_mdl(g, route_top=True).replace('StopTime "9"', 'StopTime "drone.mission(end,1)"')
powergui = "set_param('DroneFlight/powergui','SimulationMode','Discrete','SampleTime','drone.step_s');"
flight_load = ' '.join([flight_command, set_path, powergui, configure_scopes('DroneFlight/Flight scopes')])
s = s.replace('  InitFcn "setup_flight;"',
              '  PostLoadFcn '+json.dumps(flight_load)+'\n'
              +'  InitFcn '+json.dumps(flight_command))
s = s.replace('  ReturnWorkspaceOutputs on','  SaveOutput off\n  SaveTime off\n  ReturnWorkspaceOutputs off')
Path('toolbar/DroneFlight.mdl').write_text(s)
with ZipFile('EMRAXMission.slx') as src, ZipFile('toolbar/EMRAXDrive.slx','w') as dest:
    for info in src.infolist():
        data=src.read(info.filename)
        if info.filename=='simulink/blockdiagram.xml':
            tree=ET.fromstring(data)
            model = tree.find('Model')
            tree.find(".//P[@Name='PreLoadFcn']").text = ''
            ET.SubElement(model,'P',Name='PostLoadFcn').text = drive_setup + configure_scopes('EMRAXDrive/Drive results')
            tree.find(".//P[@Name='InitFcn']").text = drive_setup + drive_replay
            data=ET.tostring(tree,encoding='utf-8',xml_declaration=True)
        dest.writestr(info,data)
with ZipFile('downloads/Drone_BTP_Four_Files.zip','w',compression=ZIP_DEFLATED) as pack:
    for name in ('DroneFlight.mdl','EMRAXDrive.slx','FlightParameters.m','MotorParameters.m'):
        pack.write(Path('toolbar')/name,'Drone_BTP/'+name)
