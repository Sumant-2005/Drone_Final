"""Emit the portable text model and the equivalent R2023a native block builder."""
from pathlib import Path
import json
from collections import defaultdict

ROOT = Path(__file__).resolve().parents[1]
LIB = {
    'SubSystem':'built-in/Subsystem','Inport':'built-in/Inport','Outport':'built-in/Outport',
    'Mux':'simulink/Signal Routing/Mux','Demux':'simulink/Signal Routing/Demux',
    'FromWorkspace':'simulink/Sources/From Workspace','ToWorkspace':'simulink/Sinks/To Workspace',
    'Gain':'simulink/Math Operations/Gain','Sqrt':'simulink/Math Operations/Sqrt',
    'Saturate':'simulink/Discontinuities/Saturation','UnitDelay':'simulink/Discrete/Unit Delay',
    'MATLABFcn':'built-in/MATLABFcn','Scope':'simulink/Sinks/Scope',
    'Terminator':'simulink/Sinks/Terminator','Goto':'simulink/Signal Routing/Goto','From':'simulink/Signal Routing/From',
}

class Diagram:
    def __init__(self):
        self.blocks=[]; self.lines=defaultdict(list)

    def add(self,kind,name,x,y,w=110,h=34,**pars):
        child=Diagram() if kind=='SubSystem' else None
        self.blocks.append((kind,name,[x,y,x+w,y+h],pars,child))
        return child if child is not None else name

    def wire(self,source,target,i=1,o=1,label=''):
        self.lines[source,o].append((target,i,label))

    def function(self,name,fn,n,x,y):
        return self.add('MATLABFcn',name,x,y,158,MATLABFcn=f'{fn}(u,df_p)',OutputDimensions=str(n))

    def tap(self,src,tag,x,y):
        self.add('Goto','Store '+tag,x,y,98,GotoTag='df_'+tag,TagVisibility='global')
        self.wire(src,'Store '+tag)

    def read(self,tag,x,y):
        return self.add('From','Read '+tag,x,y,108,GotoTag='df_'+tag)

    def io(self,kind,name,x,y,port=1):
        return self.add(kind,name,x,y,40,20,Port=str(port))


def graph():
    d=Diagram()
    cmd=d.add('SubSystem','Movement command',20,90,133,64)
    ctrl=d.add('SubSystem','Flight controller',212,90,125,64)
    alloc=d.add('SubSystem','Quad X mixer',393,90,116,64)
    conv=d.add('SubSystem','Thrust to speed',575,90,124,64)
    plant=d.add('SubSystem','Rotors and body',763,90,134,64)
    scopes=d.add('SubSystem','Flight scopes',380,275,146,58)
    for left,right in [('Movement command','Flight controller'),('Flight controller','Quad X mixer'),('Quad X mixer','Thrust to speed'),('Thrust to speed','Rotors and body')]:
        d.wire(left,right)
    for src,tag,x in [('Movement command','command',100),('Flight controller','loads',285),('Quad X mixer','thrust',476),('Thrust to speed','speedRef',666),('Rotors and body','state',905)]:
        d.tap(src,tag,x,18)
    fb=d.read('state',60,195);d.wire(fb,'Flight controller',2)
    cmd.add('FromWorkspace','Forward speed command',25,28,155,VariableName='df_forward',Interpolate='off',OutputAfterFinalValue='Holding final value',SampleTime='df_p.step_s')
    cmd.add('FromWorkspace','Other mission channels',25,130,155,VariableName='df_aux',Interpolate='off',OutputAfterFinalValue='Holding final value',SampleTime='df_p.step_s')
    cmd.add('Mux','Collect commands',240,30,10,125,Inputs='2')
    cmd.io('Outport','commands',340,80)
    cmd.wire('Forward speed command','Collect commands',label='Forward speed (m per s)')
    cmd.wire('Other mission channels','Collect commands',2)
    cmd.wire('Collect commands','commands')
    for i,name in enumerate(['commands','state'],1):
        ctrl.io('Inport',name,25,38+(i-1)*70,i)
    ctrl.add('Mux','Reference and feedback',130,30,10,110,Inputs='2')
    ctrl.function('Velocity and height control','df_velocity',4,212,48)
    ctrl.add('Mux','Demands and state',468,40,10,105,Inputs='2')
    ctrl.function('Attitude and body rate control','df_moments',4,565,66)
    ctrl.io('Outport','requested loads',814,73)
    ctrl.wire('commands','Reference and feedback');ctrl.wire('state','Reference and feedback',2)
    ctrl.wire('Reference and feedback','Velocity and height control')
    ctrl.wire('Velocity and height control','Demands and state');ctrl.wire('state','Demands and state',2)
    ctrl.wire('Demands and state','Attitude and body rate control');ctrl.wire('Attitude and body rate control','requested loads')
    alloc.io('Inport','requested loads',20,75)
    alloc.add('Gain','Inverse allocation matrix',140,63,160,47,Gain='df_p.allocation',Multiplication='Matrix(K*u)')
    alloc.function('Rotor thrust limits','df_limit',4,370,66)
    alloc.io('Outport','motor thrusts',560,75)
    alloc.wire('requested loads','Inverse allocation matrix');alloc.wire('Inverse allocation matrix','Rotor thrust limits');alloc.wire('Rotor thrust limits','motor thrusts')
    conv.io('Inport','motor thrusts',20,80);conv.add('Gain','Inverse thrust coefficient',140,66,160,48,Gain='1/df_p.kT')
    conv.add('Sqrt','Square root',370,66,103,48);conv.io('Outport','speed references',550,80)
    conv.wire('motor thrusts','Inverse thrust coefficient');conv.wire('Inverse thrust coefficient','Square root');conv.wire('Square root','speed references')
    plant.io('Inport','speed references',25,140)
    plant.add('UnitDelay','Body and rotor state',30,36,128,44,InitialCondition='df_p.x0',SampleTime='df_p.step_s')
    plant.add('Mux','State and references',235,30,10,150,Inputs='2')
    plant.function('Rigid body RK4 step','df_step',16,340,95)
    plant.io('Outport','state',325,22)
    plant.wire('Body and rotor state','state');plant.wire('Body and rotor state','State and references')
    plant.wire('speed references','State and references',2);plant.wire('State and references','Rigid body RK4 step')
    plant.wire('Rigid body RK4 step','Body and rotor state')
    def sink(src,var,x,y):
        name='Log '+var;scopes.add('ToWorkspace',name,x,y,115,VariableName='df_'+var,SaveFormat='Structure With Time',MaxDataPoints='inf')
        scopes.wire(src,name)
    def scope(name,x,y):
        scopes.add('Scope',name,x,y,110,33,OpenAtSimulationStart='off')
    def split(src,name,widths,x,y,height):
        scopes.add('Demux',name,x,y,10,height,Outputs=widths);scopes.wire(src,name)
    def multitrace(src,name,labels,x,y):
        # Individually named signals provide meaningful scope legends.
        split(src,name+' channels',str(len(labels)),x,y,64)
        scopes.add('Scope',name,x+220,y,125,70,NumInputPorts=str(len(labels)),OpenAtSimulationStart='off')
        for i,lab in enumerate(labels,1):
            scopes.wire(name+' channels',name,i,i,lab)
    motors=['M1 front right','M2 back left','M3 front left','M4 back right']
    c=scopes.read('command',20,25);state=scopes.read('state',20,145)
    scopes.function('Flight measurements','df_measure',15,175,145)
    scopes.wire(state,'Flight measurements')
    split(c,'Command fields','[1 1 1 1 1]',415,15,100)
    for output,label in [(2,'Right reference'),(3,'Climb reference'),(5,'Yaw reference')]:
        scopes.add('Terminator',label,515,14+output*20,13,13)
        scopes.wire('Command fields',label,1,output)
    split('Flight measurements','Measurement fields','[1 1 2 3 4 4]',415,175,160)
    sink(c,'commandLog',180,25);sink('Flight measurements','measureLog',410,370)
    for k,name in enumerate(['Forward speed - m per s','Height - m'],1):
        scopes.add('Scope',name,712,20+82*(k-1),150,54,NumInputPorts='2',OpenAtSimulationStart='off')
        scopes.wire('Command fields',name,1,1 if k==1 else 4,'Command' if k==1 else 'Height target')
        scopes.wire('Measurement fields',name,2,k,'Actual speed' if k==1 else 'Actual height')
    for o,name in enumerate(['North and east - m','Roll pitch yaw - deg','Actual rotor speeds - rpm','Propeller loads - Nm'],3):
        scope(name,713,190+(o-3)*58);scopes.wire('Measurement fields',name,1,o)
    l=scopes.read('loads',20,462);sink(l,'loadLog',175,435)
    split(l,'Force and moments','[1 3]',414,455,55)
    scope('Collective thrust - N',715,439);scope('Roll pitch yaw moments - Nm',715,501)
    scopes.wire('Force and moments','Collective thrust - N',label='Collective (N)')
    scopes.wire('Force and moments','Roll pitch yaw moments - Nm',1,2)
    t=scopes.read('thrust',20,595);sink(t,'thrustLog',180,558);multitrace(t,'Four thrust references - N',motors,415,590)
    ref=scopes.read('speedRef',20,756);sink(ref,'refLog',177,717)
    scopes.add('Gain','Radians per second to rpm',180,805,155,Gain='30/pi');scopes.wire(ref,'Radians per second to rpm')
    multitrace('Radians per second to rpm','Four speed references - rpm',[m+' rpm' for m in motors],415,745)
    return d


def matlab_text(text):
    return "'"+text.replace("'","''")+"'"

def emit_builder(g):
    lines=['function build_drone()', '% Generated by model/generate.py. Run through run_drone from the repository root.',
           "mdl = 'DroneFlight';","folder = fullfile(fileparts(mfilename('fullpath')),'generated');",
           "if ~isfolder(folder), mkdir(folder); end","target = fullfile(folder,[mdl '.slx']);",
           "if bdIsLoaded(mdl), error('DroneFlight is already open. Close it before rebuilding.'); end",
           "if isfile(target), error('Generated model already exists. Open it with run_drone.'); end",
           "load_system('simulink');","new_system(mdl);","try",
           "set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','df_p.step_s','StopTime','9','InitFcn','df_init;','ReturnWorkspaceOutputs','on');"]
    def walk(g,sys):
        for k,name,pos,pars,child in g.blocks:
            path=sys+'/'+name
            args=[matlab_text(LIB[k]),matlab_text(path),"'Position'",str(pos).replace(',','')]
            for key,val in pars.items():args.extend((matlab_text(key),matlab_text(val)))
            lines.append('add_block('+','.join(args)+');')
            if child is not None:walk(child,path)
        for (src,o),dests in g.lines.items():
            for dest,i,label in dests:
                ins="add_line("+','.join(map(matlab_text,[sys,f'{src}/{o}',f'{dest}/{i}','autorouting','on']))+");"
                lines.append(('line = ' if label else '')+ins)
                if label:lines.append('set_param(line,\'Name\','+matlab_text(label)+');')
    walk(g,'DroneFlight')
    lines+=["save_system(mdl,target);","catch problem","    close_system(mdl,0);","    rethrow(problem);","end","end"]
    return '\n'.join(lines)+'\n'

def emit_mdl(g):
    quote=lambda s:json.dumps(s,ensure_ascii=True)
    def system(g):
        lines=['System {','  Location [20, 60, 1200, 700]']
        for k,name,pos,pars,child in g.blocks:
            blk=['Block {','  BlockType '+k,'  Name '+quote(name),'  Position '+str(pos)]
            for key,val in pars.items():blk+=['  '+key+' '+quote(val)]
            if child is not None:blk+=['  '+line for line in system(child)]
            lines+=['  '+line for line in blk+['}']]
        for (s,o),dests in g.lines.items():
            line=['Line {','  SrcBlock '+quote(s),'  SrcPort '+str(o)]
            for dest,i,label in dests:
                branch=(['Name '+quote(label)] if label else [])+['DstBlock '+quote(dest),'DstPort '+str(i)]
                line+=['  '+a for a in (branch if len(dests)==1 else ['Branch {',*['  '+a for a in branch],'}'])]
            lines+=['  '+a for a in line+['}']]
        return lines+['}']
    return '\n'.join(['Model {','  Name "DroneFlight"','  Version 8.0','  SavedCharacterEncoding "UTF-8"',
       '  SolverType "Fixed-step"','  Solver "FixedStepDiscrete"','  FixedStep "df_p.step_s"','  StopTime "9"',
       '  InitFcn "df_init;"','  ReturnWorkspaceOutputs on',*['  '+a for a in system(g)],'}',''])

if __name__=='__main__':
    g=graph()
    (ROOT/'build_drone.m').write_text(emit_builder(g))
    (ROOT/'model'/'DroneFlight.mdl').write_text(emit_mdl(g))
