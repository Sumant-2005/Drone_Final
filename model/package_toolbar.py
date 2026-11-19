"""Rebuild the four-file toolbar download from the project models."""
from xml.etree import ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED
from pathlib import Path
from generate import graph, emit_mdl
g=graph()
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
out=g.io('Outport','Motor 1 speed and load',1060,270,1)
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
s=emit_mdl(g, route_top=True).replace('StopTime "9"','StopTime "drone.mission(end,1)"').replace('  InitFcn "setup_flight;"','''  PostLoadFcn "addpath(fileparts(get_param('DroneFlight','FileName')),'-begin'); FlightParameters;"
  InitFcn "FlightParameters;"
  StartFcn "evalin('base','clear motor1_flight_replay');"''').replace('  ReturnWorkspaceOutputs on','''  ReturnWorkspaceOutputs off
  SaveFormat "StructureWithTime"
  SaveOutput on
  OutputSaveName "motor1_flight_replay"
  LimitDataPoints off
  Decimation 1
  SaveTime off''')
Path('toolbar/DroneFlight.mdl').write_text(s)
with ZipFile('EMRAXMission.slx') as src, ZipFile('toolbar/EMRAXDrive.slx','w') as dest:
    for info in src.infolist():
        data=src.read(info.filename)
        if info.filename=='simulink/blockdiagram.xml':
            tree=ET.fromstring(data)
            model = tree.find('Model')
            tree.find(".//P[@Name='PreLoadFcn']").text = ''
            ET.SubElement(model,'P',Name='PostLoadFcn').text = (
                "addpath(fileparts(get_param('EMRAXDrive','FileName')),'-begin'); MotorParameters(false);")
            tree.find(".//P[@Name='InitFcn']").text="MotorParameters;"
            data=ET.tostring(tree,encoding='utf-8',xml_declaration=True)
        dest.writestr(info,data)
with ZipFile('downloads/Drone_BTP_Four_Files.zip','w',compression=ZIP_DEFLATED) as pack:
    for name in ('DroneFlight.mdl','EMRAXDrive.slx','FlightParameters.m','MotorParameters.m'):
        pack.write(Path('toolbar')/name,'Drone_BTP/'+name)
