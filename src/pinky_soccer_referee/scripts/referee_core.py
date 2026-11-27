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
