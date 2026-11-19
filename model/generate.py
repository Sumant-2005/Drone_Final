"""Emit the portable text model and the equivalent R2023a native block builder."""
from pathlib import Path
import json
import equations
from collections import defaultdict

ROOT = Path(__file__).resolve().parents[1]
LIB = {
    'SubSystem':'built-in/Subsystem','Inport':'built-in/Inport','Outport':'built-in/Outport',
    'Mux':'simulink/Signal Routing/Mux','Demux':'simulink/Signal Routing/Demux',
    'FromWorkspace':'simulink/Sources/From Workspace',
    'Gain':'simulink/Math Operations/Gain','Sqrt':'simulink/Math Operations/Sqrt',
    'Saturate':'simulink/Discontinuities/Saturation','Integrator':'simulink/Continuous/Integrator',
    'Sum':'simulink/Math Operations/Sum','Product':'simulink/Math Operations/Product',
    'DotProduct':'simulink/Math Operations/Dot Product','MinMax':'simulink/Math Operations/MinMax',
    'Trigonometry':'simulink/Math Operations/Trigonometric Function','Constant':'simulink/Sources/Constant',
    'ZeroOrderHold':'simulink/Discrete/Zero-Order Hold',
    'Scope':'simulink/Sinks/Scope',
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
        if isinstance(source,tuple):source,o=source
        self.lines[source,o].append((target,i,label))

    def op(self,kind,name,col,row,signals,**params):
        x,y=30+col*200,40+row*108
        width = 10 if kind=='Mux' else 135
        self.add(kind,name,x,y,width,max(38,len(signals)*18),**params)
        for i,signal in enumerate(signals,1):self.wire(signal,name,i)
        return name

    def group(self,name,col,row,signals,build,**options):
        child=self.add('SubSystem',name,30+col*200,40+row*108,145,max(44,16*len(signals)))
        build(child,**options)
        for i,signal in enumerate(signals,1):self.wire(signal,name,i)
        return name

    def input(self,name,port,col,row):
        return self.io('Inport',name,30+col*200,48+row*108,port)

    def output(self,name,port,col,row,signal):
        dest=self.io('Outport',name,30+col*200,48+row*108,port)
        self.wire(signal,dest)

    def discard(self,signal,name,col,row):
        self.add('Terminator',name,30+col*200,40+row*108,14,14)
        self.wire(signal,name)

    def split(self,name,col,row,signal,widths,unused=()):
        x,y=30+col*200,40+row*108
        self.add('Demux',name,x,y,9,24*len(widths),Outputs='['+' '.join(map(str,widths))+']')
        self.wire(signal,name)
        for port in unused:
            dest=name+' unused '+str(port)
            self.add('Terminator',dest,x+68,y+24*port-18,12,12)
            self.wire((name,port),dest)
        return [(name,i) for i in range(1,len(widths)+1)]

    def mux(self,name,col,row,signals):
        return self.op('Mux',name,col,row,signals,Inputs=str(len(signals)))

    def gain(self,name,col,row,signal,k):
        return self.op('Gain',name,col,row,[signal],Gain=k)

    def matrix(self,name,col,row,signal,k):
        return self.op('Gain',name,col,row,[signal],Gain=k,Multiplication='Matrix(K*u)')

    def trig(self,name,col,row,signal,op):
        return self.op('Trigonometry',name,col,row,[signal],Operator=op)

    def prod(self,name,col,row,a,b):
        return self.op('Product',name,col,row,[a,b],Inputs='**')

    def divide(self,name,col,row,a,b):
        return self.op('Product',name,col,row,[a,b],Inputs='*/')

    def constant(self,name,col,row,value):
        return self.op('Constant',name,col,row,[],Value=value)

    def sat(self,name,col,row,signal,lo,hi):
        return self.op('Saturate',name,col,row,[signal],LowerLimit=lo,UpperLimit=hi)

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
    equations.controller(ctrl)
    alloc.io('Inport','requested loads',20,75)
    alloc.add('Gain','Inverse allocation matrix',140,63,160,47,Gain='df_p.allocation',Multiplication='Matrix(K*u)')
    equations.allocation_limit(alloc.add('SubSystem','Rotor thrust limits',370,63,135,47))
    alloc.io('Outport','motor thrusts',560,75)
    alloc.wire('requested loads','Inverse allocation matrix');alloc.wire('Inverse allocation matrix','Rotor thrust limits');alloc.wire('Rotor thrust limits','motor thrusts')
    conv.io('Inport','motor thrusts',20,80);conv.add('Gain','Inverse thrust coefficient',140,66,160,48,Gain='1/df_p.kT')
    conv.add('Sqrt','Square root',370,66,103,48);conv.io('Outport','speed references',550,80)
    conv.wire('motor thrusts','Inverse thrust coefficient');conv.wire('Inverse thrust coefficient','Square root');conv.wire('Square root','speed references')
    equations.plant(plant)
    view=scopes.add('SubSystem','Flight response',385,40,154,64)
    mixview=scopes.add('SubSystem','Mixer inputs',385,162,154,50)
    motorview=scopes.add('SubSystem','Motor outputs',385,282,154,64)
    instr=scopes.add('SubSystem','Flight instruments',165,109,138,48)
    equations.instruments(instr)
    for tag,y,dest,idx in [('command',40,'Flight response',1),('state',115,'Flight instruments',1),
                            ('loads',170,'Mixer inputs',1),('thrust',276,'Motor outputs',1),
                            ('speedRef',365,'Motor outputs',2)]:
        source=scopes.read(tag,18,y)
        scopes.wire(source,dest,idx)
    scopes.wire('Flight instruments','Flight response',2)
    scopes.wire('Flight instruments','Motor outputs',3)

    def scope(g,name,col,row,signals,labels=()):
        x,y=30+col*200,40+row*108
        g.add('Scope',name,x,y,135,max(40,16*len(signals)),NumInputPorts=str(len(signals)),OpenAtSimulationStart='off')
        for i,signal in enumerate(signals,1):
            g.wire(signal,name,i,label=labels[i-1] if labels else '')

    c=view.input('Reference',1,0,0);y=view.input('Instruments',2,0,2)
    v,_,h,_=view.split('Reference channels',1,0,c,[1,2,1,1],unused=[2,4])
    f,alt,pos,att,_=view.split('Measured channels',1,2,y,[1,1,2,3,8],unused=[5])
    scope(view,'Forward speed - m per s',3,0,[v,f],['Command','Actual'])
    scope(view,'Height - m',3,1,[h,alt],['Target height','Measured height'])
    scope(view,'North and east - m',3,2,[pos])
    scope(view,'Roll pitch yaw - deg',3,3,[att])
    l=mixview.input('Controller request',1,0,0)
    force,mom=mixview.split('Force and moments',1,0,l,[1,3])
    scope(mixview,'Collective thrust - N',2,0,[force])
    scope(mixview,'Roll pitch yaw moments - Nm',2,1,[mom])
    t=motorview.input('Thrust references',1,0,0)
    r=motorview.input('Speed references',2,0,1)
    measured=motorview.input('Instruments',3,0,2)
    motors=['M1 front right','M2 back left','M3 front left','M4 back right']
    for signal,row,name,suffix in [(t,0,'Four thrust references - N',' N'),(r,1,'Four speed references - rpm',' rpm')]:
        if row:signal=motorview.gain('Radians per second to rpm',1,row,signal,'30/pi')
        ports=motorview.split(name+' channels',2,row,signal,[1,1,1,1])
        scope(motorview,name,3,row,ports,[motor+suffix for motor in motors])
    _,speed,torque=motorview.split('Motor measurements',1,2,measured,[7,4,4],unused=[1])
    scope(motorview,'Actual rotor speeds - rpm',3,2,[speed])
    scope(motorview,'Propeller loads - Nm',3,3,[torque])
    return d


def matlab_text(text):
    return "'"+text.replace("'","''")+"'"

def emit_builder(g):
    lines=['function build_drone()', '% Generated by model/generate.py. Run through run_drone from the repository root.',
           "mdl = 'DroneFlightBlocks';","folder = fullfile(fileparts(mfilename('fullpath')),'generated');",
           "if ~isfolder(folder), mkdir(folder); end","target = fullfile(folder,[mdl '.slx']);",
           "if bdIsLoaded(mdl), error('DroneFlightBlocks is already open. Close it before rebuilding.'); end",
           "if isfile(target), error('Generated model already exists. Open it with run_drone.'); end",
           "load_system('simulink');","new_system(mdl);","try",
           "set_param(mdl,'SolverType','Fixed-step','Solver','ode4','FixedStep','df_p.step_s','StopTime','9','InitFcn','df_init;','ReturnWorkspaceOutputs','on');"]
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
    walk(g,'DroneFlightBlocks')
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
       '  SolverType "Fixed-step"','  Solver "ode4"','  FixedStep "df_p.step_s"','  StopTime "9"',
       '  InitFcn "df_init;"','  ReturnWorkspaceOutputs on',*['  '+a for a in system(g)],'}',''])

if __name__=='__main__':
    g=graph()
    (ROOT/'build_drone.m').write_text(emit_builder(g))
    (ROOT/'model'/'DroneFlight.mdl').write_text(emit_mdl(g))
