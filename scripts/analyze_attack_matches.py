#!/usr/bin/env python3
"""Re-score matches uniformly: time and fruitless circling cannot earn rewards."""
import json,math,argparse
from pathlib import Path


def analyze(directory):
    result=json.loads((directory/'result.json').read_text())
    reward=result.get('reward',{})
    if not reward:return dict(trial=directory.name,result=result['result'],reward=None)
    states={};previous={};anchors={};ref=None;times={n:[0.,0.] for n in reward}
    for line in (directory/'topics.jsonl').open():
        try:r=json.loads(line);t=r['t'];topic=r['topic'];d=r['data']
        except (ValueError,KeyError):continue
        if topic=='referee':ref=d
        elif topic.endswith('/preparation/status'):
            name=topic.split('/')[1];states[name]=d.get('state')
            if states[name]!='PLAYING':anchors.pop(name,None);previous.pop(name,None)
        elif topic in ('yellow/world','blue/world'):
            name=topic.split('/')[0];old=previous.get(name);previous[name]=t
            if states.get(name)!='PLAYING':anchors.pop(name,None);continue
            dt=max(0.,min(.2,t-old)) if old is not None else 0.
            times[name][0]+=dt
            if ref and ref.get('ball_visible'):
                x,y=ref['ball'];goal=.83 if name=='yellow' else -.83;distance=math.hypot(goal-x,y)
                stamp,anchor=anchors.get(name,(t,distance))
                if anchor-distance>.05:stamp,anchor=t,distance
                anchors[name]=stamp,anchor
                if t-stamp>5.:times[name][1]+=dt
    version=result.get('score_version',1)
    for name,s in reward.items():
        s['playing_s'],s['stagnant_ball_s']=times[name]
        if version==1:s['total']-=.2*times[name][0]+times[name][1]
    return dict(trial=directory.name,result=result['result'],score_version=2,reward=reward,quality_total=sum(s['total'] for s in reward.values()))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directories',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
    results=[analyze(Path(d)) for d in a.directories]
    Path(a.output).write_text(json.dumps(results,indent=2))
    for r in results:print(r['trial'],r['result'],'quality',round(r.get('quality_total',0),2))
