"""Optional bounded asynchronous debug recorder. No ROS dependency."""
import csv
import json
from datetime import datetime
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Thread, Event
import math
import cv2
import numpy as np


def json_safe(value):
    if isinstance(value,np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value,np.generic):
        return json_safe(value.item())
    if isinstance(value,dict):
        return {str(k):json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):
        return None
    return value


class DebugRecorder:
    def __init__(self,root,debug_log,record_video,fps=30.):
        self.directory=Path(root).expanduser()/datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.directory.mkdir(parents=True,exist_ok=False)
        self.debug_log=debug_log
        self.record_video=record_video
        self.fps=fps
        self.queue=Queue(maxsize=12)
        self.stop=Event()
        self.dropped=0
        self.error=None
        self.thread=Thread(target=self.run,name="referee-recorder",daemon=True)
        self.thread.start()

    def submit(self,metadata,images=None):
        if self.error is not None or self.stop.is_set():
            return
        if self.queue.full():
            self.dropped+=1
            return
        images={k:v.copy() for k,v in (images or {}).items() if v is not None} if self.record_video else {}
        try:
            self.queue.put_nowait((json_safe(metadata),images))
        except Full:
            self.dropped+=1

    def run(self):
        streams={}
        files=[]
        try:
            def open_file(name):
                f=(self.directory/name).open("w",encoding="utf-8",newline="")
                files.append(f)
                return f
            frames=open_file("frames.jsonl")
            tracking=performance=events=detection=None
            track_fields=["frame","t","id","visible","x","y","vx","vy","velocity_valid","heading","heading_measured","angular_velocity","team","heading_confidence","tracking_confidence","led_method","led_candidates","reacquired"]
            perf_fields=["frame","t","stage","remap_ms","detect_ms","track_ms","draw_ms","total_ms"]
            if self.debug_log:
                tracking=csv.DictWriter(open_file("tracking.csv"),fieldnames=track_fields,extrasaction="ignore")
                performance=csv.DictWriter(open_file("performance.csv"),fieldnames=perf_fields,extrasaction="ignore")
                events=csv.DictWriter(open_file("events.csv"),fieldnames=["frame","t","event","goal_color","robot_id"])
                detection=open_file("detection.jsonl")
                for writer in (tracking,performance,events):
                    writer.writeheader()
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    meta,images=self.queue.get(timeout=.1)
                except Empty:
                    continue
                try:
                    indices={}
                    for name,img in images.items():
                        shape=(img.shape[1],img.shape[0])
                        current=streams.get(name)
                        if current is None or current[1]!=shape:
                            part=0 if current is None else current[2]+1
                            if current is not None:
                                current[0].release()
                            filename=name+("" if part==0 else "_%03d"%part)+".mp4"
                            writer=cv2.VideoWriter(str(self.directory/filename),cv2.VideoWriter_fourcc(*"mp4v"),self.fps,shape)
                            if not writer.isOpened():
                                writer.release()
                                raise RuntimeError("Cannot open video writer: "+filename)
                            current=[writer,shape,part,0,filename]
                            streams[name]=current
                        current[0].write(img)
                        indices[name]={"file":current[4],"index":current[3]}
                        current[3]+=1
                    base={"frame":meta["frame"],"t":meta["t"]}
                    frames.write(json.dumps({**base,"stamp":meta.get("stamp"),"stage":meta.get("stage"),"video":indices,"error":meta.get("error")},allow_nan=False)+"\n")
                    if self.debug_log:
                        for tr in meta.get("tracks",[]):
                            tracking.writerow({**base,**tr})
                        performance.writerow({**base,"stage":meta.get("stage"),**meta.get("timings",{})})
                        for event in meta.get("events",[]):
                            events.writerow({**base,**event})
                        detection.write(json.dumps({**base,"stage":meta.get("stage"),"error":meta.get("error"),"observations":meta.get("observations",[])},allow_nan=False)+"\n")
                        if meta.get("calibration") is not None:
                            # JSON is also valid YAML 1.2; preserve each recalibration.
                            (self.directory/("calibration_%08d.yaml"%meta["frame"])).write_text(json.dumps(meta["calibration"],indent=2,allow_nan=False),encoding="utf-8")
                    if meta["frame"]%30==0:
                        for f in files:
                            f.flush()
                finally:
                    self.queue.task_done()
        except Exception as exc:
            self.error=str(exc)
        finally:
            for current in streams.values():
                current[0].release()
            for f in files:
                f.close()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=15.)
        return {"dropped_frames":self.dropped,"error":self.error,"writer_still_running":self.thread.is_alive()}
