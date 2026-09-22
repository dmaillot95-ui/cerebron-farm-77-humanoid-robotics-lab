import json,math,hashlib
from pathlib import Path
# F77 V3: real numerical controller sweep. Same reduced 3-DOF plant/disturbance as V2.
dt=0.002; T=10.0; g=9.81
candidates=[]
for kp0 in [120.,160.,220.,300.]:
 for kd0 in [24.,32.,44.,60.]:
  q=[math.radians(6),math.radians(-3),math.radians(3)]; dq=[0.,0.,0.]
  I=[8.,2.2,2.2]; kp=[kp0,55.,55.]; kd=[kd0,10.,10.]; coupling=10.; peak=abs(q[0]); effort=0.
  for k in range(int(T/dt)):
   t=k*dt; ext=[18.0 if 3.0<=t<3.08 else 0.,0.,0.]
   tau=[-kp[i]*q[i]-kd[i]*dq[i] for i in range(3)]
   tau[1]+=-coupling*((q[1]-q[0])-(q[2]-q[0])); tau[2]+=-coupling*((q[2]-q[0])-(q[1]-q[0]))
   grav=[25*g*.45*math.sin(q[0]),4*g*.38*math.sin(q[1]),4*g*.38*math.sin(q[2])]
   ddq=[(grav[i]+tau[i]+ext[i])/I[i] for i in range(3)]
   for i in range(3): dq[i]+=ddq[i]*dt; q[i]+=dq[i]*dt
   peak=max(peak,abs(q[0])); effort+=sum(x*x for x in tau)*dt
  stable=max(abs(x) for x in q)<math.radians(1) and max(abs(x) for x in dq)<.05
  candidates.append({"kp_torso":kp0,"kd_torso":kd0,"stable":stable,"final_torso_deg":math.degrees(q[0]),"peak_torso_deg":math.degrees(peak),"effort":effort})
# rank stable first, then peak+small effort regularizer
candidates.sort(key=lambda x:(not x["stable"],x["peak_torso_deg"]+1e-5*x["effort"]))
best=candidates[0]
r={"schema":"f77-controller-sweep-v3","simulation":"REAL_NUMERICAL_PARAMETER_SWEEP","candidate_count":len(candidates),"same_plant_as_v2":True,"best":best,"all_candidates":candidates,"claim_scope":"Controller search on reduced 3-DOF numerical plant only; no full humanoid/contact/HIL/physical validation."}
Path("out").mkdir(exist_ok=True); raw=json.dumps(r,sort_keys=True).encode(); r["result_sha256"]=hashlib.sha256(raw).hexdigest(); Path("out/f77-controller-sweep-v3.json").write_text(json.dumps(r,indent=2)+"\n"); print(json.dumps(best)); raise SystemExit(0 if best["stable"] else 2)
