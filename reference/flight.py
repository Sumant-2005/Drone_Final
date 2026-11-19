"""Independent flight reference for the R2023a model, in SI / NED units."""
import json
from math import atan2, cos, sin, sqrt, pi
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]

class Flight:
    def __init__(self):
        p = json.loads((ROOT/'design.json').read_text())
        self.p = p
        self.mass, self.g = p['mass_kg'], p['gravity_m_s2']
        self.dt = p['step_s']
        self.kt = p['thrust_coefficient']*p['air_density_kg_m3']*p['diameter_m']**4/(2*pi)**2
        area = pi*p['diameter_m']**2/4
        self.kq = self.kt**1.5/sqrt(2*p['air_density_kg_m3']*area)
        self.hover = sqrt(self.mass*self.g/(4*self.kt))
        self.tmax = p['sizing_weight_factor']*self.mass*self.g/4
        self.I = np.array(p['inertia_kg_m2'])
        self.rate_gain = np.array(p['rate_gain_per_s'])
        a,b = p['arm_projection_m'],self.kq/self.kt
        self.B = np.array([[1,1,1,1],[-a,a,a,-a],[a,-a,a,-a],[b,b,-b,-b]])
        self.mix = np.linalg.inv(self.B)

    def control(self,x,command,h):
        p=self.p
        f,right,up,yaw_rate=command
        c,s=cos(x[8]),sin(x[8])
        vref=np.array([c*f-s*right,s*f+c*right])
        accel=p['velocity_gain_per_s']*(vref-x[3:5])+p['linear_drag_N_per_m_s']*vref/self.mass
        accel/=max(1.,np.linalg.norm(accel)/p['max_accel_m_s2'])
        north,east=accel
        ang=np.array([atan2(-s*north+c*east,self.g),-atan2(c*north+s*east,self.g)])
        rates=np.r_[p['tilt_gain_per_s']*(ang-x[6:8]),yaw_rate]
        moments=self.I*self.rate_gain*(rates-x[9:12])+p['angular_drag_Nm_per_rad_s']*x[9:12]
        climb=up+p['height_gain_per_s']*(h+x[2])
        thrust=(self.mass*(self.g+p['climb_gain_per_s']*(climb+x[5]))+p['linear_drag_N_per_m_s']*climb)/max(.5,cos(x[6])*cos(x[7]))
        return np.r_[thrust,moments]

    def rhs(self,x,ref):
        p=self.p
        v=x[3:6]; w=x[9:12]; rotor=x[12:]
        cf,ct,cy=map(cos,x[6:9]);sf,st,sy=map(sin,x[6:9])
        z=np.array([cf*st*cy+sf*sy,cf*st*sy-sf*cy,cf*ct])
        force,*moments=self.B@(self.kt*rotor**2)
        accel=-(force*z+p['linear_drag_N_per_m_s']*v)/self.mass
        accel[2]+=self.g
        p_rate,q,r=w
        gyro=np.array([(self.I[2]-self.I[1])*q*r,(self.I[0]-self.I[2])*r*p_rate,(self.I[1]-self.I[0])*p_rate*q])
        spin=(moments-gyro-p['angular_drag_Nm_per_rad_s']*w)/self.I
        common=sf*q+cf*r
        euler=[p_rate+common*st/ct,cf*q-sf*r,common/ct]
        return np.r_[v,accel,euler,spin,(ref-rotor)/p['speed_lag_s']]

    def allocate(self,loads):
        raw=self.mix@loads
        common=np.clip(loads[0]/4,0,self.tmax)
        delta=raw-raw.mean()
        scale=1.
        if delta.max()>0:scale=min(scale,(self.tmax-common)/delta.max())
        if delta.min()<0:scale=min(scale,common/-delta.min())
        return np.clip(common+scale*delta,0,self.tmax)

    def run(self,scenario):
        profile=np.array(self.p[scenario],dtype=float);profile[:,4]*=pi/180
        dt=self.dt; idx=0; h=self.p['initial_height_m']
        x=np.r_[0,0,-h,np.zeros(9),np.full(4,self.hover)]
        samples=round(profile[-1,0]/dt)+1
        out=np.empty((samples,1+16+4+4+4+4))
        for i in range(samples):
            t=i*dt
            if idx+1<len(profile) and i==round(profile[idx+1,0]/dt):idx+=1
            cmd=profile[idx,1:]
            loads=self.control(x,cmd,h)
            thrust=self.allocate(loads)
            ref=np.sqrt(thrust/self.kt)
            out[i]=np.r_[t,x,loads,thrust,ref,self.kq*x[12:]**2]
            if i+1<samples:
                k1=self.rhs(x,ref);k2=self.rhs(x+dt*k1/2,ref);k3=self.rhs(x+dt*k2/2,ref);k4=self.rhs(x+dt*k3,ref)
                x+=dt*(k1+2*k2+2*k3+k4)/6
                h+=dt*cmd[2]
        return out

def stats(out):
    return dict(final_north_m=out[-1,1],final_east_m=out[-1,2],min_height_m=-out[:,3].max(),
                max_height_m=-out[:,3].min(),final_height_m=-out[-1,3],peak_body_forward_mps=(np.cos(out[:,9])*out[:,4]+np.sin(out[:,9])*out[:,5]).max(),
                final_speed_mps=np.linalg.norm(out[-1,4:7]),max_ref_rpm=out[:,25:29].max()*30/pi)

if __name__=='__main__':
    f=Flight()
    for scenario in ('demonstration','mission'):
        print(scenario,stats(f.run(scenario)),flush=True)
