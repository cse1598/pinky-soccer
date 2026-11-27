"""Pure OpenCV geometry, detection, tracking and conservative referee rules."""
import itertools
import math
import cv2
import numpy as np
from scipy.optimize import least_squares


def order_quad(p):
    p = np.asarray(p, np.float32).reshape(4, 2)
    s = p.sum(axis=1)
    d = p[:, 1] - p[:, 0]
    q = p[[s.argmin(), d.argmin(), s.argmax(), d.argmax()]]
    if len(np.unique(q, axis=0)) != 4 or not cv2.isContourConvex(q):
        raise ValueError("Invalid field quadrilateral")
    return q


def mask_hls(hls, low, high):
    return cv2.inRange(hls, low, high)


def contours(mask):
    return cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]


def green_quad(image):
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    m = mask_hls(hls, (35, 20, 30), (95, 230, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    cs = contours(m)
    if not cs:
        raise ValueError("Green field not found")
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < image.shape[0] * image.shape[1] * .2:
        raise ValueError("Green field too small")
    hull = cv2.convexHull(c)
    for e in np.linspace(.005, .045, 25):
        p = cv2.approxPolyDP(hull, e * cv2.arcLength(hull, True), True)
        if len(p) == 4:
            return order_quad(p)
    raise ValueError("No valid four-corner green field")


def project(p, h):
    return cv2.perspectiveTransform(np.asarray(p, np.float32).reshape(-1, 1, 2), h).reshape(-1, 2)


def robust_line(points, threshold=3.0):
    points = np.asarray(points, np.float32)
    if len(points) < 20:
        raise ValueError("Insufficient white line samples")
    rng = np.random.default_rng(37)
    best = None
    for _ in range(160):
        a, b = points[rng.choice(len(points), 2, replace=False)]
        v = b - a
        n = np.linalg.norm(v)
        if n < 30:
            continue
        normal = np.array([-v[1], v[0]]) / n
        good = np.abs((points - a) @ normal) < threshold
        if best is None or good.sum() > best.sum():
            best = good
    if best is None or best.sum() < max(20, len(points) * .45):
        raise ValueError("White line RANSAC consensus too low")
    p = points[best]
    vx, vy, x, y = cv2.fitLine(p, cv2.DIST_L2, 0, .01, .01).ravel()
    return np.array([vy, -vx, -vy*x+vx*y], float), p


def white_samples(image, green):
    # Use the green quadrilateral only to constrain the search, never as the metric reference.
    dst = np.float32([[0, 0], [999, 0], [999, 699], [0, 699]])
    h = cv2.getPerspectiveTransform(green, dst)
    m = mask_hls(cv2.cvtColor(image, cv2.COLOR_BGR2HLS), (0, 125, 0), (180, 255, 65))
    m = cv2.warpPerspective(m, h, (1000, 700), flags=cv2.INTER_NEAREST)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    samples = []
    for side in ("top", "right", "bottom", "left"):
        pts = []
        horizontal = side in ("top", "bottom")
        size, other = (1000, 700) if horizontal else (700, 1000)
        low_side = side in ("top", "left")
        # Exclude corners; sample each cross section independently.
        lo, hi = int(other*.015), int(other*.15)
        if not low_side:
            lo, hi = other-hi, other-lo
        for i in range(int(size*.06), int(size*.94), 3):
            # Goals occlude the middle of the left/right boundaries.
            if not horizontal and .30*size < i < .73*size:
                continue
            row = m[lo:hi, i] if horizontal else m[i, lo:hi]
            hits = np.flatnonzero(row)
            if not len(hits):
                continue
            splits = np.split(hits, np.where(np.diff(hits)>1)[0]+1)
            runs = [s for s in splits if 2 <= len(s) <= 24]
            if not runs:
                continue
            run = runs[0] if low_side else runs[-1]
            j = lo + float(np.mean(run))
            pts.append([i, j] if horizontal else [j, i])
        if len(pts) < 35:
            raise ValueError("White boundary missing: " + side)
        raw = project(pts, np.linalg.inv(h))
        _, good = robust_line(raw, threshold=7.)
        samples.append(good)
    return samples


def fit_radial(samples, w, h):
    # Plumb-line approximation: fixed principal point and focal scale.
    # This does not identify a full physical camera calibration.
    f = float(max(w, h))
    k = np.array([[f, 0, (w-1)/2], [0, f, (h-1)/2], [0, 0, 1]], float)
    def residual(coef):
        d = np.array([coef[0], coef[1], 0., 0., 0.])
        out = []
        for p in samples:
            u = cv2.undistortPoints(p.reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)
            v = u-u.mean(axis=0)
            _, _, axes = np.linalg.svd(v, full_matrices=False)
            out.extend((v @ axes[-1]) / math.sqrt(len(v)))
        return np.r_[out, np.asarray(coef)*2.]
    result = least_squares(residual, [0., 0.], bounds=([-0.30,-0.30],[0.30,0.30]), loss="linear", max_nfev=60)
    coef = result.x if result.success and np.linalg.norm(residual(result.x)) < np.linalg.norm(residual([0.,0.])) else [0.,0.]
    return k, np.array([coef[0],coef[1],0.,0.,0.])


def corners_from_samples(samples, k, d):
    lines = []
    for p in samples:
        u = cv2.undistortPoints(p.reshape(-1,1,2), k, d, P=k).reshape(-1,2)
        line, good = robust_line(u, 4.)
        if len(good) < 35:
            raise ValueError("White line fitting failed")
        lines.append(line)
    points = []
    for a,b in ((0,3),(0,1),(1,2),(2,3)):
        v = np.cross(lines[a], lines[b])
        if abs(v[2]) < 1e-8:
            raise ValueError("Parallel adjacent field lines")
        points.append(v[:2]/v[2])
    return order_quad(points)


class Field:
    def __init__(self, image, length, width, ppm=550., margin=.15, camera=None):
        if min(length,width,ppm,margin) <= 0:
            raise ValueError("Measured white-line dimensions and scale required")
        self.length, self.width, self.ppm = length, width, ppm
        h,w = image.shape[:2]
        green = green_quad(image)
        samples = white_samples(image, green)
        self.k, self.d = camera if camera is not None else fit_radial(samples,w,h)
        self.source = "camera_info" if camera is not None else "approximate_plumb_line"
        q = corners_from_samples(samples, self.k, self.d)
        # First construct a metric map centered on the white rectangle.
        self.homography = cv2.getPerspectiveTransform(q, np.float32([[0,0],[length*ppm,0],[length*ppm,width*ppm],[0,width*ppm]]))
        # Preserve the physical mat and goals beyond the white lines. Keep a safety margin.
        g = cv2.undistortPoints(green.reshape(-1,1,2),self.k,self.d,P=self.k).reshape(-1,2)
        outer = project(g, self.homography)
        extra = margin*ppm
        x0 = math.floor(min(0.,outer[:,0].min())-extra)
        y0 = math.floor(min(0.,outer[:,1].min())-extra)
        x1 = math.ceil(max(length*ppm,outer[:,0].max())+extra)
        y1 = math.ceil(max(width*ppm,outer[:,1].max())+extra)
        self.origin = np.array([length*ppm/2-x0,width*ppm/2-y0])
        self.size = (x1-x0+1,y1-y0+1)
        if self.size[0]*self.size[1] > 5_000_000:
            raise ValueError("Output map too large")
        shift = np.array([[1,0,-x0],[0,1,-y0],[0,0,1]],float)
        self.homography = shift @ self.homography
        self.roi = np.zeros((self.size[1],self.size[0]),np.uint8)
        cv2.fillConvexPoly(self.roi,np.round(project(g,self.homography)).astype(np.int32),255)
        # Inverse mapping: top-view pixels -> undistorted pixels -> original distorted image.
        u,v = np.meshgrid(np.arange(self.size[0],dtype=np.float32),np.arange(self.size[1],dtype=np.float32))
        p = project(np.column_stack((u.ravel(),v.ravel())),np.linalg.inv(self.homography))
        normalized = (p-self.k[:2,2])/np.array([self.k[0,0],self.k[1,1]])
        rays = np.column_stack((normalized,np.ones(len(p))))
        distorted,_ = cv2.projectPoints(rays,np.zeros(3),np.zeros(3),self.k,self.d)
        maps = distorted.reshape(self.size[1],self.size[0],2).astype(np.float32)
        self.map1,self.map2 = cv2.convertMaps(maps[:,:,0],maps[:,:,1],cv2.CV_16SC2)

    def rotate_180(self):
        """Rotate the entire rectified canvas, not merely the display."""
        w,h=self.size
        flip=np.array([[-1,0,w-1],[0,-1,h-1],[0,0,1]],float)
        self.homography=flip @ self.homography
        self.origin=np.array([w-1,h-1])-self.origin
        self.roi=cv2.flip(self.roi,-1)
        self.map1=cv2.flip(self.map1,-1)
        self.map2=cv2.flip(self.map2,-1)

    def warp(self,image):
        return cv2.remap(image,self.map1,self.map2,cv2.INTER_LINEAR)

    def metric(self, p):
        return (np.asarray(p)-self.origin)*np.array([1.,-1.])/self.ppm

    def pixel(self,p):
        return np.round(np.asarray(p)*np.array([1.,-1.])*self.ppm+self.origin).astype(int)


def blobs(mask, area_min, area_max, circularity=0.):
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    result = []
    for c in contours(mask):
        a = cv2.contourArea(c)
        if not area_min <= a <= area_max:
            continue
        perimeter = cv2.arcLength(c,True)
        if perimeter <= 0 or 4*np.pi*a/perimeter**2 < circularity:
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        result.append((np.array([m["m10"]/m["m00"],m["m01"]/m["m00"]]),a,cv2.boundingRect(c)))
    return sorted(result,key=lambda x:x[1],reverse=True)


def detect(image, field, ball_radius):
    hls = cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
    ball = mask_hls(hls,(0,95,145),(10,255,255)) | mask_hls(hls,(170,95,145),(180,255,255))
    # Expand the mat ROI to include the goal mouths.
    roi = cv2.dilate(field.roi,np.ones((max(3,int(.12*field.ppm)),)*2,np.uint8))
    ball &= roi
    expected = np.pi*(ball_radius*field.ppm)**2
    bs = blobs(ball,expected*.20,expected*4.,.45)
    dark = mask_hls(hls,(0,0,0),(180,62,255)) & field.roi
    dark = cv2.morphologyEx(dark,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
    rs = blobs(dark,.0025*field.ppm**2,.045*field.ppm**2)
    return bs,rs


def goal_intervals(image,field):
    hls = cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
    masks = {"yellow":mask_hls(hls,(12,100,105),(40,255,255)),
             "blue":mask_hls(hls,(98,45,65),(125,210,255))}
    output = {}
    for color,mask in masks.items():
        # The field x axis is image-right; goal colors are expected in their observed end zones.
        for center,area,(x,y,w,h) in blobs(mask,.003*field.ppm**2,.3*field.ppm**2):
            pos = field.metric(center)
            if abs(pos[0]) < field.length*.40 or h < w*1.4:
                continue
            ys = [field.metric((x,y))[1],field.metric((x,y+h))[1]]
            side = -1 if pos[0]<0 else 1
            if side in output:
                continue
            output[side] = (color,min(ys),max(ys))
            break
    return output


def orient_goals(field,image):
    """Require both different goal colors to establish the global frame."""
    goals=goal_intervals(field.warp(image),field)
    if set(goals)!= {-1,1} or {v[0] for v in goals.values()}!={"yellow","blue"}:
        raise ValueError("Cannot orient field: both opposite goal colors required")
    if goals[-1][0]=="blue":
        field.rotate_180()
        goals={-side:(color,-hi,-lo) for side,(color,lo,hi) in goals.items()}
    return goals


class Track:
    def __init__(self, ident):
        self.id=ident
        self.position=None
        self.velocity=np.zeros(2)
        self.stamp=None
        self.visible=False
        self.velocity_valid=False
    def update(self,p,t):
        prev=self.position
        dt=t-self.stamp if self.stamp is not None else 0.
        self.velocity_valid=bool(prev is not None and self.visible and .001<dt<.30)
        if self.velocity_valid:
            raw=(p-prev)/dt
            self.velocity=.55*self.velocity+.45*raw
        else:
            self.velocity=np.zeros(2)
        self.position=np.asarray(p,float)
        self.stamp=t
        self.visible=True
    def miss(self):
        self.visible=False
        self.velocity_valid=False


def associate(tracks, candidates, t, gate=.28):
    # Global minimum-cost assignment; never force a match through a large jump.
    available=list(range(len(candidates)))
    known=[tr for tr in tracks if tr.position is not None and t-tr.stamp<.8]
    unknown=[tr for tr in tracks if tr not in known]
    best=None
    for n in range(min(len(known),len(available)),-1,-1):
        for subset in itertools.combinations(known,n):
            for idx in itertools.permutations(available,n):
                costs=[np.linalg.norm(candidates[j]-(tr.position+tr.velocity*min(max(t-tr.stamp,0.),.15))) for tr,j in zip(subset,idx)]
                if any(c>gate for c in costs):
                    continue
                if best is None or sum(costs)<best[0]:
                    best=(sum(costs),list(zip(subset,idx)))
        if best is not None:
            break
    matches=best[1] if best else []
    used={j for tr,j in matches}
    free=sorted((j for j in available if j not in used),key=lambda j:candidates[j][0])
    matches += list(zip(unknown,free))
    for tr in tracks:
        match=next((j for tr2,j in matches if tr2 is tr),None)
        if match is None:
            tr.miss()
        else:
            tr.update(candidates[match],t)


class Rules:
    """Events are latched until reset. Unknown/occluded ball is never an OUT."""
    def __init__(self):
        self.sequence=0
        self.scores={"yellow":0,"blue":0}
        self.reset()
    def reset(self):
        self.armed=False
        self.status="WAIT_FOR_BALL"
        self.last_event=""
        self.previous=None
        self.pending=None
        self.count=0
    def update(self,p,field,radius,goals):
        if p is None:
            self.previous=None
            self.pending=None
            self.count=0
            return None
        x,y=map(float,p)
        l,w=field.length/2,field.width/2
        inside=abs(x)+radius<l and abs(y)+radius<w
        if not self.armed:
            if inside and not self.last_event:
                self.armed=True
                self.status="IN_PLAY"
                self.previous=np.array(p)
            return None
        if self.previous is None and self.pending is None:
            # Reacquisition outside is ambiguous. Require a new inside observation.
            if not inside:
                self.armed=False
                self.status="WAIT_FOR_BALL"
                return None
        candidate=None
        if abs(y)-radius>w:
            candidate=("OUT","")
        elif abs(x)-radius>l:
            side=1 if x>0 else -1
            if side not in goals:
                candidate=("UNVERIFIED_EXIT","")
            else:
                color,lo,hi=goals[side]
                # Goal mouth is approximate unless replaced by measured intervals.
                candidate=("GOAL",color) if lo+radius<=y<=hi-radius else ("OUT","")
        if candidate:
            if self.pending==candidate:
                self.count+=1
            else:
                self.pending=candidate
                self.count=1
            if self.count>=3:
                event,color=candidate
                self.sequence+=1
                if event=="GOAL":
                    self.scores[color]+=1
                self.last_event=event
                self.status=event
                self.armed=False
                return event,color
        else:
            self.pending=None
            self.count=0
        self.previous=np.array(p)
        return None


def wrap_angle(a):
    return math.atan2(math.sin(a),math.cos(a))


def iou_box(a,b):
    x,y,w,h=a
    X,Y,W,H=b
    inter=max(0.,min(x+w,X+W)-max(x,X))*max(0.,min(y+h,Y+H)-max(y,Y))
    return inter/max(w*h+W*H-inter,1.e-9)


def classify_led_pixels(bgr, blue_led_min_light=125):
    """Classify ONLY the interior of the already selected LED bar."""
    pixels=np.asarray(bgr,np.uint8).reshape(-1,3)
    if len(pixels)<8:
        return "unknown"
    h,l,s=cv2.cvtColor(pixels.reshape(-1,1,3),cv2.COLOR_BGR2HLS).reshape(-1,3).astype(float).T
    b,g,r=pixels.astype(float).T
    rg=np.minimum(r,g)
    yellow=(h>=18)&(h<=38)&(s>=145)&(l>=100)&(rg-b>=55)&(b<=.65*rg)&(np.abs(r-g)<=.35*np.maximum(r,g))
    blue=(h>=90)&(h<=135)&(s>=125)&(l>=blue_led_min_light)&(b-r>=45)&(b-g>=20)
    if np.mean(yellow)>=.75:
        return "yellow"
    if np.mean(blue)>=.70:
        return "blue"
    return "unknown"


def robot_observations(image,field,robot_blobs,blue_led_min_light=125):
    """Detect the rear transverse LED inside each dark body ROI.

    Detect bar geometry independently of team color, then classify interior.
    Body support, end geometry and tape continuation checks reject clutter.
    White/off and ambiguous colors remain UNKNOWN.
    """
    hls=cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
    result=[]
    ppm=field.ppm
    for _,area,(x,y,w,h) in robot_blobs:
        if not (.045*ppm<=min(w,h)<=.28*ppm and max(w,h)<=.32*ppm):
            continue
        pad=max(5,int(.024*ppm))
        x0,y0=max(0,x-pad),max(0,y-pad)
        x1,y1=min(image.shape[1],x+w+pad),min(image.shape[0],y+h+pad)
        a=hls[y0:y1,x0:x1]
        rgb=image[y0:y1,x0:x1]
        body=mask_hls(a,(0,0,0),(180,62,255))
        body=cv2.morphologyEx(body,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
        cs=contours(body)
        if not cs:
            continue
        c=max(cs,key=cv2.contourArea)
        p=c.reshape(-1,2).astype(float)
        if len(p)<4:
            continue
        rect=cv2.minAreaRect(c)
        center=np.array(rect[0])
        box=cv2.boxPoints(rect)
        v1,v2=box[1]-box[0],box[2]-box[1]
        axis=v1 if np.linalg.norm(v1)>=np.linalg.norm(v2) else v2
        axis=axis/max(np.linalg.norm(axis),1.e-9)
        aspect=max(rect[1])/max(min(rect[1]),1.e-9)
        # Same geometry search for white/off, yellow, and blue.
        raw_mask=np.where((a[:,:,1]>=105)|((rgb.max(axis=2)>=155)&(a[:,:,1]>=75)),255,0).astype(np.uint8)
        # Keep only near the detected chassis; this clips unrelated floor lines.
        chassis=np.zeros(a.shape[:2],np.uint8)
        cv2.drawContours(chassis,[c],-1,255,-1)
        radius=max(3,int(.025*ppm))
        support=cv2.dilate(chassis,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1)))
        mask=cv2.morphologyEx(raw_mask & support,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        candidates=[]
        for cc in contours(mask):
            ar=cv2.contourArea(cc)
            xx,yy,ww,hh=cv2.boundingRect(cc)
            if xx<=0 or yy<=0 or xx+ww>=a.shape[1] or yy+hh>=a.shape[0]:
                continue
            if not .000055*ppm**2<=ar<=.003*ppm**2:
                continue
            rr=cv2.minAreaRect(cc)
            short,long=sorted(rr[1])
            if not .034*ppm<=long<=.13*ppm or short<.006*ppm or not 2.0<=long/short<=8.0:
                continue
            fill=ar/max(short*long,1.)
            if fill<.67:
                continue
            bb=cv2.boxPoints(rr)
            z1,z2=bb[1]-bb[0],bb[2]-bb[1]
            major=z1 if np.linalg.norm(z1)>=np.linalg.norm(z2) else z2
            major=major/np.linalg.norm(major)
            normal=np.array([-major[1],major[0]])
            led=np.array(rr[0])
            delta=center-led
            distance=np.linalg.norm(delta)
            if not .025*ppm<=distance<=.14*ppm:
                continue
            if np.dot(normal,delta)<0:
                normal=-normal
            radial=float(normal@delta/distance)
            # LED long side should span the rear, not run along the chassis.
            if radial<.78 or (aspect>1.25 and abs(normal@axis)<.72):
                continue
            # Check the LED lies near the rear edge of this very body, not a
            # separate piece of white tape or a nearby goal.
            extent=(p-center)@normal
            rear=-float(extent.min())
            longitudinal=float(normal@delta)
            lateral=abs(float(major@delta))
            if abs(longitudinal-rear)>.032*ppm or lateral>.045*ppm:
                continue
            # Reject long floor tape running beyond the candidate ends.
            # A single visible continuation can occur when the robot overlaps a line.
            continuation=[]
            for sign in (-1,1):
                hits=[]
                for dpx in (.006*ppm,.012*ppm,.020*ppm):
                    pt=led+sign*major*(long/2+dpx)
                    ix,iy=np.rint(pt).astype(int)
                    hits.append(bool(raw_mask[iy,ix]) if 0<=ix<a.shape[1] and 0<=iy<a.shape[0] else True)
                continuation.append(np.mean(hits)>.65)
            if all(continuation):
                continue
            # Check that the inward edge is adjacent to dark body pixels.
            probe=led+normal*(short/2+.010*ppm)
            px,py=np.rint(probe).astype(int)
            if not (0<=px<a.shape[1] and 0<=py<a.shape[0]):
                continue
            rad=max(2,int(.012*ppm))
            nearby=body[max(0,py-rad):py+rad+1,max(0,px-rad):px+rad+1]
            if not nearby.size or np.mean(nearby>0)<.18:
                continue
            score=.36*radial+.24*fill+.22*min(long/(.065*ppm),1.)+.18*(1-min(abs(longitudinal-rear)/(.032*ppm),1.))
            # Sample the inner bar, excluding edge reflections and background.
            roi_mask=np.zeros(a.shape[:2],np.uint8)
            interior=led+(bb-led)*np.array(0.70)
            cv2.fillConvexPoly(roi_mask,np.rint(interior).astype(np.int32),255)
            color=classify_led_pixels(rgb[roi_mask>0],blue_led_min_light)
            candidates.append((score,color,normal,led))
        candidates.sort(key=lambda z:z[0],reverse=True)
        heading=None
        team="unknown"
        quality=0.
        length_m=max(rect[1])/ppm
        width_m=min(rect[1])/ppm
        if candidates:
            quality,color,normal,led=candidates[0]
            # Do not manufacture a direction if two plausible bars disagree.
            conflict=any(s>quality-.07 and (np.dot(normal,n)<.85)
                         for s,col,n,_ in candidates[1:])
            if quality>=.67 and not conflict:
                heading=math.atan2(-normal[1],normal[0])
                team=color if color in ("yellow","blue") else "unknown"
                side=np.array([-normal[1],normal[0]])
                aa=p@normal
                bb=p@side
                lo,hi=np.percentile(aa,[1,99])
                blo,bhi=np.percentile(bb,[1,99])
                # Keep body center independent of LED detection success.
                length_m=(hi-lo)/ppm
                width_m=(bhi-blo)/ppm
            else:
                quality=0.
        global_center=center+np.array([x0,y0])
        result.append({"p":field.metric(global_center),
                       "bbox":np.array([x,y,w,h],float)/ppm,
                       "heading":heading,"team":team,"quality":float(quality),
                       "length":float(length_m),"width":float(width_m)})
    return result


class RobotTrack(Track):
    def __init__(self,ident):
        super().__init__(ident)
        self.team="unknown"
        self.identity_team="unknown"
        self.color_candidate="unknown"
        self.color_count=0
        self.heading=None
        self.heading_confidence=0.
        self.angular_velocity=None
        self.bbox=None
        self.length=.16
        self.width=.12
        self.pending=None
        self.pending_count=0
        self.reacquired=False
        self.match_confidence=0.

    def observe(self,obs,t,continuous,confidence):
        old_heading=self.heading
        dt=t-self.stamp if self.stamp is not None else 0.
        had_position=self.position is not None
        if not continuous:
            self.visible=False
        super().update(obs["p"],t)
        self.heading=obs["heading"]
        self.heading_confidence=obs["quality"] if self.heading is not None else 0.
        self.angular_velocity=(wrap_angle(self.heading-old_heading)/dt
            if continuous and self.velocity_valid and self.heading is not None and old_heading is not None and .001<dt<.30 else None)
        observed=obs["team"]
        if observed=="unknown":
            self.color_candidate="unknown"
            self.color_count=0
            self.team="unknown"
            self.identity_team="unknown"
        else:
            self.color_count=self.color_count+1 if observed==self.color_candidate and continuous else 1
            self.color_candidate=observed
            self.team=observed if self.color_count>=5 else "unknown"
            self.identity_team=self.team
        self.bbox=obs["bbox"]
        self.length=obs["length"]
        self.width=obs["width"]
        self.reacquired=bool(had_position and not continuous)
        self.match_confidence=confidence
        self.pending=None
        self.pending_count=0

    def miss(self):
        super().miss()
        self.heading=None
        self.heading_confidence=0.
        self.angular_velocity=None
        self.team="unknown"
        self.color_candidate="unknown"
        self.color_count=0
        self.reacquired=False
        self.match_confidence=0.


def associate_robots(tracks,observations,t,gate=.12,reacquire_frames=2):
    """Predict + IoU + global assignment, then full-frame reacquisition.

    Detector runs globally each frame. Unmatched candidates must persist at a
    consistent location before reinitializing a lost track. No velocity is
    computed across a relocation. IDs remain provisional without colored LEDs.
    """
    def compatible(tr,o):
        return True  # color may change; never block position tracking on one LED sample
    obs=list(observations)
    known=[tr for tr in tracks if tr.visible and tr.position is not None and tr.stamp is not None and .001<t-tr.stamp<.3]
    def cost(tr,o):
        if not compatible(tr,o):
            return float("inf")
        dt=min(t-tr.stamp,.15)
        delta=tr.velocity*dt if tr.velocity_valid else np.zeros(2)
        distance=np.linalg.norm(o["p"]-(tr.position+delta))
        if distance>gate:
            return float("inf")
        pred=tr.bbox.copy()
        pred[:2]+=delta*np.array([1.,-1.])
        overlap=iou_box(pred,o["bbox"])
        mismatch=tr.identity_team!="unknown" and o["team"]!="unknown" and tr.identity_team!=o["team"]
        value=.72*distance/gate+.28*(1-overlap)+(.10 if mismatch else 0.)
        return value if value<=.85 else float("inf")
    best=None
    for n in range(min(len(known),len(obs)),-1,-1):
        for subset in itertools.combinations(known,n):
            for ids in itertools.permutations(range(len(obs)),n):
                costs=[cost(tr,obs[j]) for tr,j in zip(subset,ids)]
                if not all(math.isfinite(c) for c in costs):
                    continue
                if best is None or sum(costs)<best[0]:
                    best=(sum(costs),list(zip(subset,ids,costs)))
        if best is not None:
            break
    matched=set()
    used=set()
    for tr,j,c in best[1] if best else []:
        tr.observe(obs[j],t,True,max(0.,1-c))
        matched.add(tr)
        used.add(j)
    lost=[tr for tr in tracks if tr not in matched]
    free=[j for j in range(len(obs)) if j not in used]
    for tr in lost:
        tr.miss()
    # Match remaining observations to lost IDs by known team, then by
    # pending candidate proximity. If both IDs are anonymous, identity after
    # relocation is inherently uncertain, so left-to-right is only a fallback.
    for tr in sorted(lost,key=lambda z:(z.identity_team=="unknown",z.id)):
        candidates=[j for j in free if compatible(tr,obs[j])]
        if not candidates:
            tr.pending=None
            tr.pending_count=0
            continue
        def rank(j):
            o=obs[j]
            team_penalty=0 if tr.identity_team!="unknown" and o["team"]==tr.identity_team else 1
            distance=np.linalg.norm(o["p"]-tr.pending["p"]) if tr.pending is not None else (np.linalg.norm(o["p"]-tr.position) if tr.position is not None else o["p"][0])
            return team_penalty,distance
        j=min(candidates,key=rank)
        free.remove(j)
        o=obs[j]
        consistent=(tr.pending is not None and
                    np.linalg.norm(o["p"]-tr.pending["p"])<=gate and
                    (tr.pending["team"]=="unknown" or o["team"]=="unknown" or tr.pending["team"]==o["team"]))
        tr.pending_count=tr.pending_count+1 if consistent else 1
        tr.pending=o
        if tr.pending_count>=max(1,reacquire_frames):
            tr.observe(o,t,False,.5)


def trapezoid_points(track,field):
    """Short edge faces the measured heading; no direction arrow needed."""
    theta=track.heading
    e=np.array([math.cos(theta),math.sin(theta)])
    n=np.array([-e[1],e[0]])
    c=track.position
    half=max(.025,track.length/2)+.005
    rear=max(.025,track.width/2)+.005
    front=.66*rear
    return np.array([field.pixel(c+half*e-front*n),field.pixel(c+half*e+front*n),
                     field.pixel(c-half*e+rear*n),field.pixel(c-half*e-rear*n)],np.int32)
