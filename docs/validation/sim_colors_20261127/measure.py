"""Recompute fixed interior ROI means from the saved, unmodified camera PNGs."""
from pathlib import Path
import cv2,numpy as np,json
root=Path(__file__).resolve().parent
targets={'ball':([230,57,4],[7,117,246]),'yellow':([237,229,4],[29,121,247]),'blue':([5,75,134],[104,70,237])}
rows=[]
y,x=np.ogrid[:480,:640]
for engine in ['mujoco','gazebo']:
 for view in ['blue','yellow']:
  rgb=cv2.cvtColor(cv2.imread(str(root/f'{engine}_{view}.png')),cv2.COLOR_BGR2RGB)
  for name in ([view,'ball'] if view=='blue' else [view]):
   mask=((x-320)**2+(y-249)**2<=30**2) if name=='ball' else ((x>=215)&(x<425)&(y>=175)&(y<200))
   mean=rgb[mask].mean(0); hls=cv2.cvtColor(rgb,cv2.COLOR_RGB2HLS)[mask].mean(0)
   rows.append(dict(engine=engine,object=name,file=f'{engine}_{view}.png',roi='circle center=(320,249), radius=30' if name=='ball' else 'x=[215,425), y=[175,200)',pixels=int(mask.sum()),rgb=mean.tolist(),hls=hls.tolist(),delta_rgb=(mean-targets[name][0]).tolist(),delta_hls=(hls-targets[name][1]).tolist()))
(root/'measurements.json').write_text(json.dumps(rows,indent=2)+'\n')
for r in rows: print(r['engine'],r['object'],'RGB',np.round(r['rgb'],2),'HLS',np.round(r['hls'],2),'delta RGB',np.round(r['delta_rgb'],2),'delta HLS',np.round(r['delta_hls'],2))
