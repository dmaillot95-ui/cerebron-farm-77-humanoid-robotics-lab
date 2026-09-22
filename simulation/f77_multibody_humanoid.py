import json, math, hashlib
from pathlib import Path
# Reduced multibody humanoid: torso + two hip/leg oscillators with coupled PD stabilization.
dt=0.002; T=10.0; g=9.81
# q=[torso,left_hip,right_hip], dq; simplified coupled inertias and gravity terms.
q=[math.radians(6), math.radians(-3), math.radians(3)]; dq=[0.,0.,0.]
I=[8.0,2.2,2.2]; kp=[95.,42.,42.]; kd=[22.,8.,8.]; coupling=8.0
peak=[abs(x) for x in q]; effort=0.; samples=[]
for k in range(int(T/dt)):
 t=k*dt
 ext=[18.0 if 3.0<=t<3.08 else 0.,0.,0.]
 # desired symmetric stance = zero; coupling hips to torso and to each other
 tau=[-kp[i]*q[i]-kd[i]*dq[i] for i in range(3)]
 tau[1]+=-coupling*((q[1]-q[0])-(q[2]-q[0])); tau[2]+=-coupling*((q[2]-q[0])-(q[1]-q[0]))
 grav=[25*g*0.45*math.sin(q[0]), 4*g*0.38*math.sin(q[1]), 4*g*0.38*math.sin(q[2])]
 ddq=[(grav[i]+tau[i]+ext[i])/I[i] for i in range(3)]
 for i in range(3): dq[i]+=ddq[i]*dt; q[i]+=dq[i]*dt; peak[i]=max(peak[i],abs(q[i])); effort+=tau[i]*tau[i]*dt
 if k%100==0: samples.append([round(t,3),*q,*dq])
stable=max(abs(x) for x in q)<math.radians(1) and max(abs(x) for x in dq)<0.05
r={"schema":"f77-multibody-humanoid-v2","simulation":"REAL_NUMERICAL_SIMULATION","model":"reduced_3dof_coupled_humanoid","dt_s":dt,"duration_s":T,"metrics":{"stable":stable,"final_deg":[math.degrees(x) for x in q],"peak_deg":[math.degrees(x) for x in peak],"control_effort":effort},"samples":samples,"claim_scope":"Reduced 3-DOF multibody numerical dynamics; not full humanoid rigid-body/contact physics or physical validation."}
Path("out").mkdir(exist_ok=True); raw=json.dumps(r,sort_keys=True).encode(); r["result_sha256"]=hashlib.sha256(raw).hexdigest(); Path("out/f77-multibody.json").write_text(json.dumps(r,indent=2)+"\n"); print(json.dumps(r["metrics"])); raise SystemExit(0 if stable else 2)
