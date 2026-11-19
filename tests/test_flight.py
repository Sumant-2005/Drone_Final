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
        print(f"{name}: final N/E/H {s['final_north_m']:.4f}, {s['final_east_m']:.4f}, {s['final_height_m']:.4f} m | PASS")
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
            if kind=='FromWorkspace':
                assert pars['Interpolate']=='off'
                assert pars['OutputAfterFinalValue']=='Holding final value'
            if child is not None:
                inspect(child);inputs=sum(b[0]=='Inport' for b in child.blocks)
            elif kind in ('Inport','From','FromWorkspace'): inputs=0
            elif kind=='Mux': inputs=int(pars['Inputs'])
            elif kind=='Scope':inputs=int(pars.get('NumInputPorts',1))
            else: inputs=1
            assert all((name,i+1) in used for i in range(inputs))
    inspect(d)
