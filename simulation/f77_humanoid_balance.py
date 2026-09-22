import json,math,hashlib
from pathlib import Path
# Real numerical simulation: linearized sagittal inverted-pendulum + PD ankle control.
dt=0.002; T=8.0; h=0.85; g=9.81; kp=35.0; kd=9.0
# initial 5 degree lean; state theta, omega
theta=math.radians(5.0); omega=0.0; peak=abs(theta); energy=0.0
series=[]
for k in range(int(T/dt)):
    t=k*dt
    disturbance=0.9 if 2.0<=t<2.08 else 0.0  # rad/s^2 impulse-like perturbation
    u=-kp*theta-kd*omega
    alpha=(g/h)*theta + u + disturbance
    omega += alpha*dt; theta += omega*dt
    peak=max(peak,abs(theta)); energy += u*u*dt
    if k%100==0: series.append([round(t,3),theta,omega,u])
result={"schema":"f77-humanoid-balance-sim-v1","simulation":"REAL_NUMERICAL_SIMULATION","model":"linearized_inverted_pendulum_PD","dt_s":dt,"duration_s":T,"initial_lean_deg":5.0,"disturbance":{"start_s":2.0,"duration_s":0.08,"accel_rad_s2":0.9},"metrics":{"final_angle_deg":math.degrees(theta),"peak_angle_deg":math.degrees(peak),"control_effort_integral":energy,"stable":abs(theta)<math.radians(1) and abs(omega)<0.05},"samples":series,"claim_scope":"Actual numerical dynamics integration, not a high-fidelity humanoid, Isaac/MuJoCo run, HIL, or physical robot test."}
Path("out").mkdir(exist_ok=True); raw=json.dumps(result,sort_keys=True).encode(); result["result_sha256"]=hashlib.sha256(raw).hexdigest(); Path("out/f77-simulation.json").write_text(json.dumps(result,indent=2)+"\n"); print(json.dumps(result["metrics"]))
if not result["metrics"]["stable"]: raise SystemExit(2)
