# -*- coding: utf-8 -*-
"""
=======================================================================
  Compliant Finger - 20R PRBM with Tendon-Wrapping Cylinders
  (arm26-style <geom> wrapping in spatial tendons)
=======================================================================
"""

import numpy as np
import mujoco
import mujoco.viewer
import time

# ========================= Geometry (SI) ===========================
N_LINKS = 4               
L_BASE  = 25.00e-3        
L_LNK   = 25.00e-3        
H_LNK   = 10.00e-3        
W       = 5.00e-3         

L_GAP = 15.00e-3          
T_GAP = [3.00e-3, 3.25e-3, 3.25e-3, 3.50e-3]         
H_CABLE = 6.00e-3         

# ========================= PRBM Physics ============================
N_SUB = 20                

# Base stiffness properties corresponding to a nominal 3.25mm joint
JS_BASE = 0.05            
JD_BASE = 0.005           
T_NOMINAL = 3.25e-3       

MAX_ANGLE = 0.80          
RANGE_SUB = MAX_ANGLE / N_SUB  

MAX_PUSH = 15.0           

# ========================= Colors ==================================
C_BASE = "0.25 0.25 0.25 1"    
C_LINK = "0.95 0.55 0.10 1"    
C_CABL = "1.00 0.85 0.10 1"    
C_WRAP = "0.30 0.30 0.80 0.25" 

def make_xml():
    L = []
    A = L.append

    A('<mujoco model="compliant_finger_wrap_tendons">')
    A('  <compiler angle="radian" autolimits="true"/>')
    A('  <option timestep="0.002" gravity="0 0 0" integrator="implicit"/>')
    A('')
    A('  <visual>')
    A('    <headlight diffuse="0.75 0.75 0.75" ambient="0.35 0.35 0.35" specular="0 0 0"/>')
    A('    <rgba haze="0.15 0.19 0.27 1"/>')
    A('  </visual>')
    A('')
    A('  <default>')
    A(f'    <joint type="hinge" axis="0 1 0" limited="true" range="0 {RANGE_SUB:.4f}"/>')
    A(f'    <geom rgba="{C_LINK}"/>')
    A('    <default class="solid">')
    A('      <geom contype="1" conaffinity="1"/>') 
    A('    </default>')
    A('    <default class="flex">')
    A('      <geom contype="0" conaffinity="0"/>') 
    A('    </default>')
    A('    <default class="wrap">')
    A(f'      <geom contype="0" conaffinity="0" mass="0" group="3" rgba="{C_WRAP}"/>')
    A('    </default>')
    A('  </default>')
    A('')
    A('  <worldbody>')
    A('    <light pos="0.12 -0.20 0.30" dir="-0.3 0.5 -1.0" diffuse="0.9 0.9 0.9"/>')
    A('    <light pos="0.12  0.20 -0.20" dir="-0.3 -0.5 1.0" diffuse="0.4 0.4 0.4"/>')
    A('')

    ind_level = 2
    def get_ind(): return '  ' * ind_level

    # --- BASE LINK ---
    A(get_ind() + '<body name="base" pos="0 0 0">')
    ind_level += 1
    A(get_ind() + f'<geom name="g_base" class="solid" type="box" pos="{L_BASE/2:.5f} 0 {H_LNK/2:.5f}" size="{L_BASE/2:.5f} {W:.5f} {H_LNK/2:.5f}" rgba="{C_BASE}"/>')
    A(get_ind() + f'<site name="t1_start" pos="{L_BASE:.5f} 0 {H_CABLE:.5f}" size="0.0015"/>')

    # --- KINEMATIC CHAIN (Flexures + Links) ---
    for i in range(N_LINKS):
        dl = L_GAP / N_SUB  
        
        # Calculate EXACT stiffness for this specific gap based on (t / t_nominal)^3
        stiffness_factor = (T_GAP[i] / T_NOMINAL)**3
        
        # Total gap stiffness
        js_gap_total = JS_BASE * stiffness_factor
        jd_gap_total = JD_BASE * stiffness_factor
        
        # Sub-segment stiffness (springs in series must be stiffer)
        js_sub = js_gap_total * N_SUB
        jd_sub = jd_gap_total * N_SUB

        # Wrapping cylinder radius: distance from hinge axis to cable path
        wrap_r = H_CABLE - T_GAP[i] / 2

        # 20 Sub-segments per gap
        for j in range(N_SUB):
            if i == 0 and j == 0:
                pos_x = L_BASE
            elif j == 0:
                pos_x = L_LNK
            else:
                pos_x = dl

            A(get_ind() + f'<body name="link{i+1}_f{j+1}" pos="{pos_x:.5f} 0 0">')
            ind_level += 1

            # Individualized hinge with mathematically correct stiffness
            A(get_ind() + f'<joint name="j{i+1}_{j+1}" pos="0 0 {T_GAP[i]/2:.5f}" stiffness="{js_sub:.5f}" damping="{jd_sub:.5f}"/>')
            
            # Flexure sub-segment visual
            A(get_ind() + f'<geom name="g_f{i+1}_{j+1}" class="flex" type="box" pos="{dl/2:.5f} 0 {T_GAP[i]/2:.5f}" size="{dl/2:.5f} {W:.5f} {T_GAP[i]/2:.5f}"/>')
            
            # Wrapping cylinder at the hinge axis (Y-oriented, collision-free, hidden in group 3)
            A(get_ind() + f'<geom name="wrap_{i+1}_{j+1}" class="wrap" type="cylinder" '
              f'pos="0 0 {T_GAP[i]/2:.5f}" euler="1.5708 0 0" size="{wrap_r:.5f} {W:.5f}"/>')
            
            # Tendon routing site at midpoint of sub-segment
            A(get_ind() + f'<site name="t{i+1}_r{j+1}" pos="{dl/2:.5f} 0 {H_CABLE:.5f}" size="0.001"/>')

        # Solid link at the end of the 20th flexure segment
        A(get_ind() + f'<body name="link{i+1}" pos="{dl:.5f} 0 0">')
        ind_level += 1
        A(get_ind() + f'<geom name="g_l{i+1}" class="solid" type="box" pos="{L_LNK/2:.5f} 0 {H_LNK/2:.5f}" size="{L_LNK/2:.5f} {W:.5f} {H_LNK/2:.5f}"/>')
        
        A(get_ind() + f'<site name="t{i+1}_end" pos="0 0 {H_CABLE:.5f}" size="0.0015"/>')
        if i < N_LINKS - 1:
            A(get_ind() + f'<site name="t{i+2}_start" pos="{L_LNK:.5f} 0 {H_CABLE:.5f}" size="0.0015"/>')

    # Close all nested bodies cleanly
    while ind_level > 2:
        ind_level -= 1
        A(get_ind() + '</body>')

    A('  </worldbody>')
    A('')

    # --- ROUTED TENDON SEGMENTS (arm26-style wrapping) ---
    # Each tendon: start_site -> [wrap_geom -> mid_site] x N_SUB -> end_site
    # The <geom> element between two <site> elements tells MuJoCo to wrap
    # the tendon around that cylinder when the straight path would penetrate it.
    A('  <tendon>')
    for i in range(N_LINKS):
        A(f'    <spatial name="cable{i+1}" width="0.0015" rgba="{C_CABL}" stiffness="0" damping="0">')
        A(f'      <site site="t{i+1}_start"/>')
        for j in range(N_SUB):
            A(f'      <geom geom="wrap_{i+1}_{j+1}"/>')
            A(f'      <site site="t{i+1}_r{j+1}"/>')
        A(f'      <site site="t{i+1}_end"/>')
        A('    </spatial>')
    A('  </tendon>')
    A('')

    # --- MOTORS ---
    A('  <actuator>')
    for i in range(N_LINKS):
        A(f'    <motor name="motor{i+1}" tendon="cable{i+1}" ctrllimited="true" ctrlrange="-{MAX_PUSH:.1f} 0"/>')
    A('  </actuator>')
    A('')
    A('</mujoco>')

    return '\n'.join(L)

PROFILE = [
    (1.0,  0.0),            
    (3.5,  -MAX_PUSH),      
    (2.0,  -MAX_PUSH),      
    (3.5,  0.0),            
    (1.0,  0.0),            
]
T_TOTAL = sum(p[0] for p in PROFILE)

def force_target(t):
    t = t % T_TOTAL
    prev = PROFILE[0][1]
    for dur, end_val in PROFILE:
        if t <= dur:
            a = (t / dur) if dur > 0 else 1.0
            a = a * a * (3.0 - 2.0 * a) 
            return prev + (end_val - prev) * a
        t    -= dur
        prev  = end_val
    return 0.0

def run():
    print("=" * 62)
    print(f"  Compliant Finger - Wrap-Tendon Model (arm26-style)")
    print("=" * 62)

    XML = make_xml()
    model = mujoco.MjModel.from_xml_string(XML)
    data  = mujoco.MjData(model)

    j_ids = []
    for i in range(1, N_LINKS + 1):
        for j in range(1, N_SUB + 1):
            j_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"j{i}_{j}"))

    dt = model.opt.timestep
    STEPS_PER_SYNC = 8
    sim_t = 0.0
    t_last_print = -1.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance  = 0.45
        viewer.cam.azimuth   = 20.0
        viewer.cam.elevation = 15.0 
        viewer.cam.lookat[:] = [0.10, 0.0, -0.05]

        while viewer.is_running():
            t0 = time.time()
            for _ in range(STEPS_PER_SYNC):
                data.ctrl[:] = force_target(sim_t)
                mujoco.mj_step(model, data)
                sim_t += dt
            
            viewer.sync()
            spare = STEPS_PER_SYNC * dt - (time.time() - t0)
            if spare > 0:
                time.sleep(spare)

            if sim_t - t_last_print >= 0.5:
                t_last_print = sim_t
                qs = [data.qpos[model.jnt_qposadr[j]] for j in j_ids]
                
                gap_bends = [sum(qs[i*N_SUB:(i+1)*N_SUB]) for i in range(N_LINKS)]
                curl = np.degrees(sum(gap_bends))
                ctrl = data.ctrl[0]
                
                print(
                    f"  t={sim_t:6.1f}s | push force={ctrl:+5.1f} N | curl={curl:+7.1f} deg\n"
                    f"      3.00mm base : {np.degrees(gap_bends[0]):+5.1f} deg\n"
                    f"      3.25mm mid 1: {np.degrees(gap_bends[1]):+5.1f} deg\n"
                    f"      3.25mm mid 2: {np.degrees(gap_bends[2]):+5.1f} deg\n"
                    f"      3.50mm tip  : {np.degrees(gap_bends[3]):+5.1f} deg\n"
                )

if __name__ == "__main__":
    run()