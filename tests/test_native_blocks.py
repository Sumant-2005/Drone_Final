"""Evaluate the wired native math on independent states, including off the mission axes."""
import json
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'model'),str(ROOT/'reference')]
from generate import graph
from flight import Flight

def evaluate(diagram, inputs, constants, states=None):
    """Discrete algebra at an instant. Integrator outputs are supplied by name."""
    blocks={b[1]:b for b in diagram.blocks}
    parents={name:{} for name in blocks}
    for source,edges in diagram.lines.items():
        for dest,port,_ in edges:parents[dest][port]=source
    cache={}
    def get(name):
        if name in cache:return cache[name]
        kind,_,_,pars,child=blocks[name]
        if kind=='Integrator':
            out=states[name]
        elif kind=='Inport':
            out=inputs[int(pars['Port'])-1]
        else:
            args=[get(s)[o-1] for _,(s,o) in sorted(parents[name].items())]
            if child is not None:
                out, nested_rates=evaluate(child,args,constants,states)
                derivatives.update(nested_rates)
                cache[name]=out
                return out
            if kind=='Demux':
                widths=list(map(int,pars['Outputs'].strip('[]').split()))
                assert sum(widths)==args[0].size
                cache[name]=np.split(args[0],np.cumsum(widths)[:-1])
                return cache[name]
            if kind=='Constant':out=constants[pars['Value']]
            elif kind=='Mux':out=np.concatenate(args)
            elif kind in ('Outport','ZeroOrderHold'):out=args[0]
            elif kind=='Sqrt':out=np.sqrt(args[0])
            elif kind=='Trigonometry':out={'sin':np.sin,'cos':np.cos,'atan':np.arctan,'tan':np.tan}[pars['Operator']](args[0])
            elif kind=='Sum':out=sum(a if sign=='+' else -a for a,sign in zip(args,pars['Inputs']))
            elif kind=='Gain':
                val=constants[pars['Gain']]
                out=val@args[0] if pars.get('Multiplication')=='Matrix(K*u)' else val*args[0]
            elif kind=='Product':out=args[0]*args[1] if pars['Inputs']=='**' else args[0]/args[1]
            elif kind=='DotProduct':out=np.dot(*args)
            elif kind=='Saturate':out=np.clip(args[0],constants[pars['LowerLimit']],constants[pars['UpperLimit']])
            elif kind=='MinMax':out=(np.min if pars['Function']=='min' else np.max)(args)
            else:raise AssertionError(kind)
        cache[name]=[np.atleast_1d(out)]
        return cache[name]
    derivatives={}
    ports=sorted((int(b[3]['Port']),b[1]) for b in diagram.blocks if b[0]=='Outport')
    result=[get(name)[0] for _,name in ports]
    for kind,name,*_ in diagram.blocks:
        if kind=='Integrator':
            src,o=parents[name][1]
            derivatives[name]=get(src)[o-1]
    return result,derivatives

def block_constants(f):
    p=f.p
    constants={f'df_p.{key}':value for key,value in p.items()}
    constants.update({'1':1.,'inf':np.inf,'0':0.,'-1':-1.,'eps':np.finfo(float).eps,
        '0.5':.5,'df_p.max_thrust':f.tmax,'df_p.kT':f.kt,'df_p.kQ':f.kq,
        '30/pi':30/np.pi,'180/pi':180/np.pi,'df_p.allocation':f.mix,'df_p.effectiveness':f.B,
        '1/df_p.kT':1/f.kt,'ones(1,4)/4':np.ones((1,4))/4,
        'diag(df_p.inertia_kg_m2(:).*df_p.rate_gain_per_s(:))':np.diag(f.I*f.rate_gain),
        'diag(1./df_p.inertia_kg_m2(:))':np.diag(1/f.I),
        'df_p.gyro_gain':np.diag(np.roll(f.I,1)-np.roll(f.I,-1)),
        '[0 1;-1 0]':np.array([[0,1],[-1,0]]),'1/df_p.gravity_m_s2':1/f.g,
        '1/df_p.max_accel_m_s2':1/p['max_accel_m_s2'],
        'df_p.mass_kg*df_p.climb_gain_per_s':f.mass*p['climb_gain_per_s'],
        'df_p.mass_kg*df_p.gravity_m_s2':f.mass*f.g,'1/df_p.mass_kg':1/f.mass,
        '1/df_p.speed_lag_s':1/p['speed_lag_s'],'[0;0;df_p.gravity_m_s2]':[0,0,f.g],
        'df_p.linear_drag_N_per_m_s/df_p.mass_kg':p['linear_drag_N_per_m_s']/f.mass})
    return constants

def test_native_control_allocator_body_and_scope_math():
    f=Flight()
    constants=block_constants(f)
    systems={b[1]:b[-1] for b in graph().blocks if b[-1] is not None}
    instruments=next(b[-1] for b in systems['Flight scopes'].blocks if b[1]=='Flight instruments')
    rng=np.random.default_rng(302)
    for _ in range(24):
        x=rng.uniform(-.4,.4,16)
        x[2]-=5;x[12:]+=f.hover
        command=rng.uniform(-2,2,4);h=5.2
        signals,_=evaluate(systems['Flight controller'],[np.r_[command[:3],h,command[3]],x],constants)
        np.testing.assert_allclose(signals[0],f.control(x,command,h),atol=1e-10)
        thrust,_=evaluate(systems['Quad X mixer'],signals,constants)
        np.testing.assert_allclose(thrust[0],f.allocate(signals[0]),atol=1e-10)
        ref,_=evaluate(systems['Thrust to speed'],thrust,constants)
        np.testing.assert_allclose(ref[0],np.sqrt(thrust[0]/f.kt))
        states=dict(zip(['Position','Velocity','Euler angles','Body rates','Rotor speed'],np.split(x,[3,6,9,12])))
        state,dx=evaluate(systems['Rotors and body'],ref,constants,states)
        np.testing.assert_array_equal(state[0],x)
        np.testing.assert_allclose(np.concatenate([dx[k] for k in states]),f.rhs(x,ref[0]),atol=1e-9)
        measurement,_=evaluate(instruments,[x],constants)
        expected=np.r_[np.cos(x[8])*x[3]+np.sin(x[8])*x[4],-x[2],x[:2],x[6:9]*180/np.pi,x[12:]*30/np.pi,f.kq*x[12:]**2]
        np.testing.assert_allclose(measurement[0],expected,atol=1e-10)

def test_wired_forward_flight():
    f=Flight()
    constants=block_constants(f)
    systems={b[1]:b[-1] for b in graph().blocks if b[-1] is not None}
    x=np.r_[0.,0.,-5.,np.zeros(9),np.full(4,f.hover)]
    profile=np.array(f.p['demonstration'])
    golden=json.loads((ROOT/'reference'/'checkpoints.json').read_text())['demonstration']
    sample={round(t/f.dt):i for i,t in enumerate(golden['time_s'])}
    groups=['Position','Velocity','Euler angles','Body rates','Rotor speed']

    def rhs(z,reference):
        states=dict(zip(groups,np.split(z,[3,6,9,12])))
        _,dx=evaluate(systems['Rotors and body'],[reference],constants,states)
        return np.concatenate([dx[key] for key in groups])

    last=round(profile[-1,0]/f.dt)
    peak=0.
    for n in range(last+1):
        t=n*f.dt
        forward=profile[np.searchsorted(profile[:,0],t,side='right')-1,1]
        command=np.array([forward,0.,0.,5.,0.])
        loads,_=evaluate(systems['Flight controller'],[command,x],constants)
        thrust,_=evaluate(systems['Quad X mixer'],loads,constants)
        (ref,),_=evaluate(systems['Thrust to speed'],thrust,constants)
        if n in sample:
            j=sample[n]
            np.testing.assert_allclose(x[:3],golden['position'][j],atol=1e-7)
            np.testing.assert_allclose(thrust[0],golden['thrust'][j],atol=1e-7)
            np.testing.assert_allclose(x[12:]*30/np.pi,golden['rotor_rpm'][j],atol=1e-6)
        peak=max(peak,x[3])
        if n<last:
            k1=rhs(x,ref);k2=rhs(x+f.dt*k1/2,ref)
            k3=rhs(x+f.dt*k2/2,ref);k4=rhs(x+f.dt*k3,ref)
            x=x+f.dt*(k1+2*k2+2*k3+k4)/6
    print(f'Wired forward demo in Python: north {x[0]:.4f} m | height {-x[2]:.4f} m | peak speed {peak:.4f} m/s | PASS')
