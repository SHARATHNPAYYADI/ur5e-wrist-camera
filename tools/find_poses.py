"""Find 5 UR5e joint poses from which the wrist camera sees the cube.

Forward kinematics follows ur_macro.xacro (ur5e default_kinematics.yaml) plus the
camera_joint / camera_optical_joint from ur_isaac.urdf.xacro. Frames: base_link.
"""
import numpy as np
from scipy.optimize import least_squares

def rx(a): c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a): c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a): c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def T(xyz, rpy=(0,0,0), q=0.0):
    R = rz(rpy[2]) @ ry(rpy[1]) @ rx(rpy[0]) @ rz(q)   # joint rotates about its own z
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = xyz; return M
pi = np.pi
FIX = [  # (xyz, rpy) per joint, base_link_inertia -> wrist_3_link
    ((0,0,0.1625),(0,0,0)),
    ((0,0,0),(pi/2,0,0)),
    ((-0.425,0,0),(0,0,0)),
    ((-0.3922,0,0.1333),(0,0,0)),
    ((0,-0.0997,0),(pi/2,0,0)),
    ((0,0.0996,0),(pi/2,pi,pi)),
]
BASE = T((0,0,0),(0,0,pi))
FLANGE = T((0,0,0),(0,-pi/2,-pi/2)) @ T((0,0,0),(pi/2,0,pi/2))   # wrist_3 -> tool0
CAM = T((0,-0.06,0.04),(1.5708,-1.5708,0)) @ T((0,0,0),(-1.5708,0,-1.5708))  # tool0 -> optical

def fk(q):
    """Return list of link-frame transforms and camera optical frame, all in base_link."""
    M = BASE.copy(); frames = []
    for (xyz,rpy),qi in zip(FIX,q):
        M = M @ T(xyz,rpy,qi); frames.append(M.copy())
    tool0 = M @ FLANGE
    return frames, tool0 @ CAM, tool0

# camera intrinsics (matches the USD camera in spawn_ur5e.py)
W,H = 640,480
F = 18.15*W/20.955; CX,CY = W/2, H/2
CUBE = np.array([0.5,0.0,0.05]); EDGE = 0.1
CORNERS = np.array([CUBE+0.5*EDGE*np.array([sx,sy,sz]) for sx in(-1,1) for sy in(-1,1) for sz in(-1,1)])

def project(Tbc, p):
    pc = np.linalg.inv(Tbc) @ np.append(p,1)
    return pc[:3], (F*pc[0]/pc[2]+CX, F*pc[1]/pc[2]+CY) if pc[2]>1e-6 else (None)

HOME = np.array([0,-1.57,0,-1.57,0,0.0])
LO = np.array([-pi,-pi,-pi,-pi,-pi,-pi]); HI = -LO

def residual(q, cam_pos, w_pos=1.0):
    _, Tc, _ = fk(q)
    z = Tc[:3,2]; x = Tc[:3,0]; pos = Tc[:3,3]
    to_cube = CUBE-pos; d = np.linalg.norm(to_cube); to_cube/=d
    return np.concatenate([w_pos*(pos-cam_pos),            # camera where we want it
                           0.5*(z-to_cube),                # looks at cube centre
                           [0.1*x[2]],                     # horizon level
                           0.01*(q-HOME)])                 # stay near home

def check(q):
    frames, Tc, Tool = fk(q)
    # crude collision / sanity: arm joints above the floor, nothing inside the cube box
    nodes = [f[:3,3] for f in frames]+[Tool[:3,3],Tc[:3,3]]
    pts = []
    for a,b in zip(nodes[:-1],nodes[1:]):
        pts += [a+(b-a)*t for t in np.linspace(0,1,8)]
    pts = np.array(pts)
    ok_floor = bool(np.all(pts[8:,2]>0.04))            # skip the base segment
    inside = bool(np.any(np.all(np.abs(pts-CUBE)<EDGE/2+0.05,axis=1)))
    proj = [project(Tc,c) for c in CORNERS]
    if any(p[1] is None for p in proj): return False, None
    uv = np.array([p[1] for p in proj]); depth = min(p[0][2] for p in proj)
    margin = min(uv[:,0].min(), uv[:,1].min(), W-uv[:,0].max(), H-uv[:,1].max())
    return (ok_floor and not inside and depth>0.12), dict(margin=margin, depth=depth,
            centre=project(Tc,CUBE)[1], width=uv[:,0].max()-uv[:,0].min())

TARGETS = {
 "top-down":      (0.50, 0.00, 0.40),
 "from robot":    (0.25, 0.00, 0.35),
 "from left":     (0.50, 0.30, 0.25),
 "from right":    (0.50,-0.30, 0.25),
 "far side":      (0.75, 0.00, 0.30),
}
if __name__ == "__main__":
    f,Tc,Tt = fk(HOME)
    print("home tool0 xyz",Tt[:3,3].round(3)," cam xyz",Tc[:3,3].round(3)," cam z axis",Tc[:3,2].round(2))
    rng = np.random.default_rng(0)
    for name,pos in TARGETS.items():
        best=None
        for k in range(60):
            q0 = HOME if k==0 else HOME+rng.normal(0,0.9,6)
            r = least_squares(residual,np.clip(q0,LO,HI),args=(np.array(pos),),bounds=(LO,HI))
            ok,info = check(r.x)
            err = np.linalg.norm(residual(r.x,np.array(pos))[:9])
            if ok and info["margin"]>25 and err<0.02 and np.max(np.abs(r.x-HOME))<2.6:
                cost = np.linalg.norm(r.x-HOME)
                if best is None or cost<best[0]: best=(cost,r.x,info,err)
        if best is None: print(f"{name:11s} NO SOLUTION"); continue
        _,q,info,err = best
        print(f"{name:11s} q={np.round(q,3).tolist()}  cube centre px=({info['centre'][0]:.0f},{info['centre'][1]:.0f}) "
              f"width={info['width']:.0f}px margin={info['margin']:.0f}px depth={info['depth']:.2f}m err={err:.4f}")
