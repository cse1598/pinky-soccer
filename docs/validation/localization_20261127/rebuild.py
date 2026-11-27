"""Rebuild the study tables/CSV/plots from the preserved raw evaluator logs."""
from pathlib import Path
import csv,json,math,sys,gzip,shlex
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from benchmark_localization import summarize, is_localizer_command
runs=['baseline_mujoco_valid','baseline_gazebo_valid','mujoco_release_drive','mujoco_corner','mujoco_far_goal','gazebo_aligned','rectangle_gazebo_probe','rectangle_gazebo_rate05','mujoco_strict_hls','mujoco_curve_opponent']
out=Path(__file__).resolve().parent; base=out/'raw'
results={};allrows=[]
for run in runs:
 r=json.loads(gzip.decompress((base/(run+'.json.gz')).read_bytes())); summary=summarize(r)
 errors=[]
 for row in r['samples']:
  e=np.asarray(row['pose'])-row['truth'];e[2]=math.atan2(math.sin(e[2]),math.cos(e[2]))
  allrows.append([run,row['source'],row['t'],*row['pose'],*row['truth'],*e,row['sync_error']]);errors.append(e)
 if errors:
  e=np.array(errors);bad=np.abs(e[:,2])>math.pi/2;summary['wrong_pi_branch_events']=int(np.count_nonzero(bad & ~np.r_[False,bad[:-1]]))
  source= 'rectangle' if any(s['source']=='rectangle' for s in r['samples']) else 'amcl'
  p=np.array([s['pose'] for s in r['samples'] if s['source']==source]);d=np.diff(p,axis=0);d[:,2]=np.arctan2(np.sin(d[:,2]),np.cos(d[:,2]));summary['pose_jump_events']=int(np.count_nonzero((np.linalg.norm(d[:,:2],axis=1)>.25)|(np.abs(d[:,2])>.7)))
 cpus={}
 for row in r['cpu']:
  if is_localizer_command(row.get('argv',shlex.split(row['cmd']))):cpus.setdefault(row['pid'],[]).append(row)
 usage=[]
 for pid,entries in cpus.items():
  if len(entries)>1:
   a,b=entries[0],entries[-1];usage.append(100*(b['cpu']-a['cpu'])/(b['elapsed']-a['elapsed']))
 summary['localization_cpu_percent_one_core']=sum(usage)
 summary['cpu_components']=len(usage)
 states=r['states']; first=next((s for s in states if s.get('ready')),None)
 summary['initial_rotation_imu_rad']=first['rotation_rad'] if first else None
 summary['final_commands']=r.get('commands')
 summary['last_truth']=r.get('truth_samples',[])[-1:] or None
 if summary['last_status']:
  summary['last_status'].pop('records',None)
 results[run]=summary
 print(run,summary['ready_wall_s'],summary['sources'], 'cpu',sum(usage),'rotation',summary['initial_rotation_imu_rad'],'jumps',summary.get('pose_jump_events'))
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
with (out/'poses.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['run','source','stamp','x','y','yaw','true_x','true_y','true_yaw','error_x','error_y','error_yaw','time_pair_difference']);w.writerows(allrows)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(11,6.5),sharex='col')
for col,sim in enumerate(['mujoco','gazebo']):
 for label in ([f'baseline_{sim}_valid','mujoco_release_drive'] if sim=='mujoco' else [f'baseline_{sim}_valid','gazebo_aligned']):
  rows=[x for x in allrows if x[0]==label];a=np.asarray([x[2:] for x in rows],dtype=float);t=a[:,0]-a[0,0]
  axes[0,col].plot(t,1000*np.hypot(a[:,7],a[:,8]),label=label)
  axes[1,col].plot(t,np.rad2deg(a[:,9]),label=label)
 axes[0,col].set_title(sim+(' (Gazebo new: aligned static only)' if sim=='gazebo' else ' (same two GoToPose requests)'))
 axes[0,col].set_ylabel('Position error [mm]');axes[1,col].set_ylabel('Yaw error [deg]');axes[1,col].set_xlabel('Time since first estimate [sim s]')
 for ax in axes[:,col]:ax.grid(alpha=.3);ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(out/'pose_errors.png',dpi=150);plt.close(fig)
