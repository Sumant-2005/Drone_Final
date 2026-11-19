import json
import sys
from pathlib import Path
from math import pi, sqrt
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'reference'))
sys.path.insert(0,str(ROOT/'model'))
from flight import Flight, stats
from generate import graph, emit_builder, emit_mdl

def test_report_trim_and_power():
    f=Flight()
    for factor in (1.,1.1):
        thrust=factor*400*9.81/4
        rpm=60*sqrt(thrust/(.1*1.225*1.**4))
        omega=sqrt(thrust/f.kt)
        ideal=thrust**1.5/sqrt(2*1.225*(pi/4))
        assert omega*30/pi==pytest.approx(rpm)
        assert f.kq*omega**3==pytest.approx(ideal)
        assert ideal>20000
        print(f'{factor:.1f} weight: {thrust:.2f} N/rotor | {rpm:.2f} rpm | {ideal/1000:.2f} kW ideal/rotor | PASS')

def test_individual_axes_and_allocation_limits():
    f=Flight()
    hover=f.mass*f.g
    for i,signs in enumerate(([-1,1,1,-1],[1,-1,1,-1],[1,1,-1,-1])):
        req=np.array([hover,0.,0.,0.]);req[i+1]=1
        thrust=f.allocate(req)
        np.testing.assert_allclose(f.B@thrust,req,atol=1e-10)
        assert np.array_equal(np.sign(thrust-hover/4),signs)
    for req in ([hover,0,0,1e4],[-200,2,3,1],[1e5,0,400,1]):
        thrust=f.allocate(req)
        assert min(thrust)>=0 and max(thrust)<=f.tmax
        assert sum(thrust)==pytest.approx(np.clip(req[0],0,4*f.tmax))

def test_flight_scenarios():
    f=Flight()
    saved={}
    checkpoints={}
    for name in ('demonstration','mission'):
        out=f.run(name)
        t=out[:,0];x=out[:,1:17];load=out[:,17:21];T=out[:,21:25];ref=out[:,25:29];q=out[:,29:33]
        s=stats(out);saved[name]=s
        assert np.isfinite(out).all() and np.all(q>0)
        np.testing.assert_allclose(T.sum(axis=1),np.clip(load[:,0],0,4*f.tmax),atol=1e-7)
        np.testing.assert_allclose(f.kt*ref**2,T,atol=1e-9)
        np.testing.assert_allclose(q,f.kq*x[:,12:]**2,atol=1e-9)
        if name=='demonstration':
            assert np.max(abs(x[t<=1,:3]-[0,0,-5]))<1e-9
            assert 7.5<s['final_north_m']<8.5 and s['final_speed_mps']<.1
            assert 1.9<s['peak_body_forward_mps']<2.2
            assert np.max(abs(x[:,2]+5))<.1
            # At onset the rear pair rises. At release the front pair brakes.
            for time,sgn in [(1,-1),(5,1)]:
                z=T[round(time/f.dt)]
                assert (z[0]-z[1])*sgn>0 and (z[2]-z[3])*sgn>0
        else:
            assert len(t)==round(52/f.dt)+1 and np.linalg.norm(x[-1,:3]-[0,0,-5])<.5
            assert np.max(x[:,0])>6 and np.max(x[:,1])>6 and s['max_height_m']>7.5
            assert s['min_height_m']>2 and np.max(x[:,8])>np.deg2rad(15)
        # At each command edge and within each leg, compare the complete motor
        # response as well as the path. All times land on the solver grid.
        edges=np.array(f.p[name])[:,0]
        times=np.unique(np.minimum(np.r_[edges,edges+.25,edges+.75],edges[-1]))
        index=np.rint(times/f.dt).astype(int)
        checkpoints[name]={'time_s':times.tolist()}
        for key,values in [('position',x[:,:3]),('velocity',x[:,3:6]),('attitude',x[:,6:9]),
                           ('thrust',T),('ref_rpm',ref*30/pi),('rotor_rpm',x[:,12:]*30/pi)]:
            checkpoints[name][key]=values[index].round(8).tolist()
        print(f"{name}: final N/E/H {s['final_north_m']:.4f}, {s['final_east_m']:.4f}, {s['final_height_m']:.4f} m | PASS")
    golden=ROOT/'reference'/'checkpoints.json'
    if golden.exists():
        stored=json.loads(golden.read_text())
        for name,arrays in checkpoints.items():
            for key,values in arrays.items():
                np.testing.assert_allclose(values,stored[name][key],atol=1e-6,rtol=1e-8)
    else:
        golden.write_text(json.dumps(checkpoints,separators=(',',':'))+'\n')
    # Stable precision across platforms and a compact independent MATLAB fixture.
    fixture=ROOT/'reference'/'expected.json'
    rounded={k:{n:round(float(v),8) for n,v in values.items()} for k,values in saved.items()}
    if fixture.exists():
        recorded=json.loads(fixture.read_text())
        for case in rounded:
            for signal in rounded[case]:
                assert rounded[case][signal]==pytest.approx(recorded[case][signal],abs=1e-6,rel=1e-8)
    else:
        fixture.write_text(json.dumps(rounded,indent=2)+'\n')

def test_model_sources_agree_and_have_connected_inputs():
    d=graph()
    assert (ROOT/'model'/'DroneFlight.mdl').read_text()==emit_mdl(d)
    assert (ROOT/'build_drone.m').read_text()==emit_builder(d)
    def inspect(diagram):
        names={b[1] for b in diagram.blocks}
        assert len(names)==len(diagram.blocks)
        used=set()
        for (src,_),targets in diagram.lines.items():
            assert src in names
            for target,port,_ in targets:
                assert target in names and (target,port) not in used
                used.add((target,port))
        for kind,name,_,pars,child in diagram.blocks:
            assert kind not in ('ToWorkspace','MATLABFcn','Fcn')
            if kind=='FromWorkspace':
                assert pars['Interpolate']=='off'
                assert pars['OutputAfterFinalValue']=='Holding final value'
            if child is not None:
                inspect(child);inputs=sum(b[0]=='Inport' for b in child.blocks)
            elif kind in ('Inport','From','FromWorkspace','Constant'): inputs=0
            elif kind=='Mux': inputs=int(pars['Inputs'])
            elif kind in ('Sum','Product'):inputs=len(pars['Inputs'])
            elif kind=='DotProduct':inputs=2
            elif kind=='MinMax':inputs=int(pars['Inputs'])
            elif kind=='Scope':inputs=int(pars.get('NumInputPorts',1))
            else: inputs=1
            assert all((name,i+1) in used for i in range(inputs))
    inspect(d)
    # Check routing independently of display labels: these five signals feed
    # the controller feedback, scopes and the motor replay.
    root_ports = {'Flight command':'flight_command','Flight controller':'thrust_and_moments',
                  'Motor mixer':'motor_thrust','Thrust to speed':'motor_speed_reference',
                  'Drone dynamics':'flight_state'}
    blocks={b[1]:b for b in d.blocks}
    for source,tag in root_ports.items():
        sinks=[blocks[target] for target,_,_ in d.lines[source,1] if blocks[target][0]=='Goto']
        assert len(sinks)==1 and sinks[0][3]['GotoTag']==tag
    views=blocks['Flight scopes'][-1]
    expected_tags={'Flight response':{1:'flight_command'},'Mixer inputs':{1:'thrust_and_moments'},
                   'Motor outputs':{1:'motor_thrust',2:'motor_speed_reference'},
                   'Flight measurements':{1:'flight_state'}}
    for b in views.blocks:
        if b[0]=='From':
            for dest,port,_ in views.lines[b[1],1]:
                assert expected_tags[dest].pop(port)==b[3]['GotoTag']
    assert all(not ports for ports in expected_tags.values())
    for generated in (emit_builder(d),emit_mdl(d)):
        assert 'df_' not in generated and 'setup_flight' in generated


def test_mission_with_alternate_integrator():
    # Preserve the controller sampling rate. Only change the plant integration:
    # two explicit midpoint substeps in place of the model's single RK4 step.
    f=Flight()
    profile=np.array(f.p['mission'],dtype=float)
    profile[:,4]=np.deg2rad(profile[:,4])
    golden=json.loads((ROOT/'reference'/'checkpoints.json').read_text())['mission']
    checkpoints={round(t/f.dt):j for j,t in enumerate(golden['time_s'])}
    height=f.p['initial_height_m']
    state=np.r_[0.,0.,-height,np.zeros(9),np.full(4,f.hover)]
    substep=f.dt/2
    position_error=0.
    rpm_error=0.
    last=round(profile[-1,0]/f.dt)
    for n in range(last+1):
        command=profile[np.searchsorted(profile[:,0],n*f.dt,side='right')-1,1:]
        load=f.control(state,command,height)
        reference=np.sqrt(f.allocate(load)/f.kt)
        if n in checkpoints:
            j=checkpoints[n]
            position_error=max(position_error,np.linalg.norm(state[:3]-golden['position'][j]))
            rpm_error=max(rpm_error,np.max(abs(state[12:]*30/pi-golden['rotor_rpm'][j])))
        if n<last:
            height+=command[2]*f.dt
            for _ in range(2):
                midpoint=state+substep/2*f.rhs(state,reference)
                state+=substep*f.rhs(midpoint,reference)
    assert position_error<.01 and rpm_error<1.
    print(f'Alternate integration, {len(checkpoints)} mission checkpoints: max position difference '
          f'{position_error:.7f} m | max rotor difference {rpm_error:.5f} rpm | PASS')
