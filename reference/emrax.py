"""Average-voltage dq reference for the motor replay, using peak phase amperes.

The PWM drive in Simulink resolves the switches. This reference uses their
fundamental voltage and therefore predicts mean tracking rather than ripple.
"""
from math import cos, hypot, pi, sqrt, tan
import numpy as np


def motor_replay(f, mission, p, subdivisions=50):
    step = f.dt / subdivisions
    pole_pairs, flux, rs, ld, lq = (p[k] for k in ('pole_pairs','flux','Rs','Ld','Lq'))
    inertia = p['rotor_inertia'] + p['propeller_inertia']
    voltage = p['Vdc']/2
    current_limit = sqrt(2)*p['continuous_phase_arms']
    kt = 1.5*pole_pairs*flux
    bw = 2*pi*p['fsw']/10
    a = tan(65*pi/180) + 1/cos(65*pi/180)
    tau = 1/(bw/100*a)
    total_tau = tau+1/bw
    kp = inertia/(kt*a*total_tau)
    ki = kp/(a*a*total_tau)
    # Work directly in V rather than modulation units.
    kp_d, kp_q, ki_current = bw*ld, bw*lq, bw*rs
    start = p['ramp_seconds']+p['settle_seconds']
    warm_t = np.arange(round(start/f.dt))*f.dt
    fraction = np.minimum(warm_t/p['ramp_seconds'],1)
    command = np.r_[f.hover*fraction, mission[:,25]]
    load = np.r_[f.kq*(f.hover*fraction)**2,mission[:,29]]
    out = np.empty((len(command),8))
    omega = filt = i_d = i_q = int_speed = int_d = int_q = 0.
    for n in range(len(command)):
        ref, torque_load = float(command[n]),float(load[n])
        for j in range(subdivisions):
            error = ref-filt
            iq_ref = min(current_limit,max(-current_limit,kp*error+int_speed))
            electrical = pole_pairs*omega
            vd = max(-voltage,min(voltage,-kp_d*i_d+int_d-electrical*lq*i_q))
            vq = max(-voltage,min(voltage,kp_q*(iq_ref-i_q)+int_q+electrical*(flux+ld*i_d)))
            scale = max(1,hypot(vd,vq)/voltage)
            torque = 1.5*pole_pairs*(flux+(ld-lq)*i_d)*i_q
            if j==0:
                out[n] = (n*f.dt-start,ref*30/pi,omega*30/pi,torque_load,torque,iq_ref,i_q,i_d)
            did = (vd/scale-rs*i_d+electrical*lq*i_q)/ld
            diq = (vq/scale-rs*i_q-electrical*(flux+ld*i_d))/lq
            int_d = max(-voltage,min(voltage,int_d-step*ki_current*i_d))
            int_q = max(-voltage,min(voltage,int_q+step*ki_current*(iq_ref-i_q)))
            int_speed = max(-current_limit,min(current_limit,int_speed+step*ki*error))
            filt += step*(omega-filt)/tau
            omega += step*(torque-torque_load)/inertia
            i_d += step*did
            i_q += step*diq
    return out
