#!/usr/bin/env python3
"""
PID‐based rotation with live plot for OPTA actuator,
printing the absolute error (mm) and effort (%) each loop,
and treating ctrl.get_position_mm() as the absolute length.
"""

import os, sys, time, math
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from controller.opta_controller import OPTAController

# — PID gains (tune these) —
Kp     = 10.0
Ki     = 0.5
Kd     = 0.0
Offset = 980.0    # mm offset to map ADC=0 to your actual start length


relay_str = "grouser" # what actuator do we want to control

# — Rotation profile settings —
ANGLE_DEG     = 90.0   # degrees
DURATION_S    = 10    # seconds
POLL_INTERVAL = 0.01   # seconds

# — Geometry constants —
A = 955.9   # mm
B = 247.5   # mm

def rotate_with_pid_and_plot(ctrl, angle_deg, duration, poll_interval,relay_str):
    ω = math.radians(angle_deg) / duration

    # 1) latch forward relay once and zero speed
    ctrl.open_relay_fwd("grouser")
    ctrl.coast(relay_str)

    # 2) setup live plot
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, duration)
    Lmin = math.hypot(A - B, 0)
    Lmax = math.hypot(A + B, 0)
    ax.set_ylim(Lmin * 0.9, Lmax * 1.1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Length (mm)")
    sp_line, = ax.plot([], [], "r-", label="Set-point")
    ac_line, = ax.plot([], [], "b-", label="Actual")
    ax.legend()
    fig.canvas.draw()

    ts, sps, acts = [], [], []
    t0     = time.time()
    last_e = 0.0
    integ  = 0.0
    t_last = t0

    while True:
        now = time.time()
        t   = now - t0
        dt  = now - t_last
        if dt <= 0:
            time.sleep(poll_interval)
            continue

        # desired absolute length
        L_sp = math.hypot(
            A + B * math.sin(ω * t),
            B * math.cos(ω * t)
        )

        # actual absolute length from ADC + Offset
        L_act = ctrl.get_position_mm(relay_str) + Offset

        # PID
        error     = L_sp - L_act
        integ    += error * dt
        derivative = (error - last_e) / dt
        u = Kp * error + Ki * integ + Kd * derivative

        # clamp forward‐only effort to [0…100]%
        u = max(0.0, min(100.0, u))
    


        # apply: only modulate voltage, relay held closed
        if u <= 0.0:
            ctrl.coast(relay_str)
        else:
            ctrl.set_speed_percent(u,relay_str)

        # record & update plot
        ts.append(t);  sps.append(L_sp);  acts.append(L_act)
        sp_line.set_data(ts, sps)
        ac_line.set_data(ts, acts)
        if t > ax.get_xlim()[1]:
            ax.set_xlim(0, t + duration * 0.1)
        ax.figure.canvas.draw()
        plt.pause(0.001)

        # print error & effort
        print(f"t={t:5.2f}s | err={error:7.2f} mm | effort={u:6.2f}%   ",
              end="\r", flush=True)

        if t >= duration:
            break

        last_e = error
        t_last = now
        time.sleep(poll_interval)

    # final stop: zero & open relays
    ctrl.stop(relay_str)
    plt.ioff()
    plt.show()


def main():
    ctrl = OPTAController("COM3")
    rotate_with_pid_and_plot(ctrl, ANGLE_DEG, DURATION_S, POLL_INTERVAL,relay_str)

if __name__ == "__main__":
    main()
