#!/usr/bin/env python3
"""Render existing MuJoCo lamp materials; no simulation/control input is generated."""
import argparse
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--two-robots', action='store_true')
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    scene = args.scene
    if args.two_robots:
        import shutil
        from pinky_soccer_sim.mujoco_multi import place_robots
        from pinky_soccer_sim.led_visual import led_materials
        scene = args.output/'two_robots.xml'
        shutil.copy(args.scene, scene)
        place_robots(scene, (-.4, 0., 0.), (.2, 0., np.pi))
        led_materials(scene)
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    physical={k:getattr(model,k).copy() for k in ('body_mass','body_inertia','geom_size',
        'geom_friction','geom_contype','geom_conaffinity','dof_damping','jnt_range')}
    mats=[i for i in range(model.nmat) if 'soccer_led' in
          (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MATERIAL,i) or '')]
    if not mats:
        raise ValueError('Run current MuJoCo launch first: no dedicated LED materials')
    renderer=mujoco.Renderer(model,480,640)
    camera=mujoco.MjvCamera()
    camera.lookat[:]=data.xpos[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')]
    camera.lookat[0]-=.04
    camera.lookat[2]+=.07
    camera.azimuth,camera.elevation,camera.distance=180.,-20.,.35
    reference=None
    differences={}
    panels=[]
    for label,rgb in [('OFF',(0,0,0)),('GREEN',(0,96,0)),('YELLOW',(96,96,0)),
                      ('BLUE',(0,0,96)),('RED',(96,0,0))]:
        for mat in mats:
            model.mat_rgba[mat]=[*[v/255. for v in rgb],1.]
        renderer.update_scene(data,camera='front_camera')
        front=renderer.render().copy()
        if reference is None:
            reference=front
        hls = cv2.cvtColor(front[130:], cv2.COLOR_RGB2HLS)
        counts = {name: int(cv2.countNonZero(cv2.inRange(hls, np.array(low), np.array(high))))
            for name, low, high in [
                ('yellow_common', (24,85,180), (35,180,255)),
                ('blue_common', (96,30,135), (115,130,255)),
                ('yellow_sim', (24,60,100), (35,180,255)),
                ('blue_sim', (96,20,80), (115,130,255)),
                ('orange_strong', (3,15,100), (19,220,255)),
                ('orange_weak', (3,65,60), (19,160,99))]}
        cv2.imwrite(str(args.output/('front_'+label.lower()+'.png')), cv2.cvtColor(front, cv2.COLOR_RGB2BGR))
        differences[label]=dict(hls_pixels=counts, changed_pixels=int(np.count_nonzero(np.any(front!=reference,axis=2))),
            max_rgb_difference=int(np.max(abs(front.astype(int)-reference.astype(int)))))
        renderer.update_scene(data,camera=camera)
        external=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
        cv2.putText(external,label,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
        cv2.imwrite(str(args.output/(label.lower()+'.png')),external)
        panels.append(cv2.resize(external,(320,240)))
    renderer.close()
    physical_equal={k:bool(np.array_equal(v,getattr(model,k))) for k,v in physical.items()}
    result=dict(scene=str(args.scene),front_camera_changes=differences,physical_arrays_equal=physical_equal)
    (args.output/'render.json').write_text(json.dumps(result,indent=2))
    cv2.imwrite(str(args.output/'lamp_colors.png'),np.concatenate(panels,axis=1))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
