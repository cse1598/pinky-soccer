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
        # Keep curved sections for distortion validation, not just RANSAC inliers.
        line, _ = robust_line(raw, threshold=7.)
        near = raw[np.abs(raw @ line[:2] + line[2]) <= 14.]
        coordinate = near[:,0] if horizontal else near[:,1]
        full = raw[:,0] if horizontal else raw[:,1]
        if len(near)<35 or np.ptp(coordinate)<.85*np.ptp(full):
            raise ValueError("Incomplete white boundary: " + side)
        samples.append(near)
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
            u = cv2.undistortPoints(p.astype(np.float64).reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)
            v = u-u.mean(axis=0)
            _, _, axes = np.linalg.svd(v, full_matrices=False)
            out.extend((v @ axes[-1]) / math.sqrt(len(v)))
        return np.r_[out, np.asarray(coef)*2.]
    result = least_squares(residual, [0., 0.], bounds=([-0.30,-0.30],[0.30,0.30]), loss="soft_l1", f_scale=.2, max_nfev=100)
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


def boundary_errors(samples,k,d,h,length_px,width_px):
    """All retained samples, including points excluded by line fitting."""
    errors=[]
    for i,p in enumerate(samples):
        u=cv2.undistortPoints(p.astype(np.float64).reshape(-1,1,2),k,d,P=k)
        uv=project(u,h)
        axis=1 if i%2==0 else 0
        target=(0.,length_px,width_px,0.)[i]
        errors.append(float(np.percentile(np.abs(uv[:,axis]-target),90)))
    return errors


def select_geometry(samples,w,h,length_px,width_px,camera=None,mode="auto",max_error_px=5.):
    f=float(max(w,h))
    nominal=np.array([[f,0,(w-1)/2],[0,f,(h-1)/2],[0,0,1]],float)
    options=[]
    if mode in ("auto","none"):
        options.append(("none",nominal,np.zeros(5)))
    if mode in ("auto","plumb_line"):
        k,d=fit_radial(samples,w,h)
        options.append(("approximate_plumb_line",k,d))
    if camera is not None and mode in ("auto","camera_info"):
        options.append(("camera_info",*camera))
    results=[]
    for source,k,d in options:
        try:
            q=corners_from_samples(samples,k,d)
            if abs(cv2.contourArea(q)) < .15*w*h:
                continue
            dst=np.float32([[0,0],[length_px,0],[length_px,width_px],[0,width_px]])
            H=cv2.getPerspectiveTransform(q,dst)
            errors=boundary_errors(samples,k,d,H,length_px,width_px)
            if not np.isfinite(errors).all():
                continue
            # Worst-side error matters: don't improve top/bottom by bending a side.
            score=max(errors)+.25*np.mean(errors)
            results.append((score,source,k,d,H,errors))
        except (ValueError,cv2.error,np.linalg.LinAlgError):
            continue
    if not results:
        raise ValueError("No usable calibration model")
    results.sort(key=lambda x:x[0])
    best=results[0]
    # Prefer factory calibration or no extra distortion if nearly equivalent.
    for preferred in ("camera_info","none"):
        candidates=[r for r in results if r[1]==preferred and r[0]<=best[0]+.25 and max(r[5])<=max_error_px]
        if candidates:
            best=candidates[0]
            break
    if max(best[5])>max_error_px:
        raise ValueError("White boundary p90 error too high: "+str([(r[1],np.round(r[5],2).tolist()) for r in results]))
    return best[1:]


def verify_rectified_boundary(image,field,max_error_px=5.):
    """Independent check against actual tape in the output raster."""
    white=mask_hls(cv2.cvtColor(image,cv2.COLOR_BGR2HLS),(0,115,0),(180,255,75))
    L,W=field.length/2,field.width/2
    corners=np.array([field.pixel((-L,W)),field.pixel((L,W)),field.pixel((L,-W)),field.pixel((-L,-W))])
    errors=[]
    for i in range(4):
        a,b=corners[i],corners[(i+1)%4]
        direction=(b-a)/np.linalg.norm(b-a)
        normal=np.array([-direction[1],direction[0]])
        offsets=np.arange(-max(12,int(.045*field.ppm)),max(12,int(.045*field.ppm))+1)
        values=[]
        total=0
        for t in np.linspace(.08,.92,100):
            if i%2 and .28<t<.74:
                continue
            total+=1
            p=a+t*(b-a)
            xy=np.rint(p+offsets[:,None]*normal).astype(int)
            if np.any(xy<0) or np.any(xy[:,0]>=white.shape[1]) or np.any(xy[:,1]>=white.shape[0]):
                continue
            hits=np.flatnonzero(white[xy[:,1],xy[:,0]])
            runs=[r for r in np.split(hits,np.where(np.diff(hits)>1)[0]+1) if 2<=len(r)<=max(8,int(.035*field.ppm))]
            if runs:
                run=min(runs,key=lambda r:abs(np.mean(offsets[r])))
                values.append(abs(float(np.mean(offsets[run]))))
        if len(values)<.60*total:
            raise ValueError("Rectified boundary missing on side "+str(i))
        errors.append(float(np.percentile(values,90)))
    if max(errors)>max_error_px:
        raise ValueError("Rectified tape alignment p90: "+str(np.round(errors,2)))
    return errors


class Field:
    def __init__(self, image, length, width, ppm=550., margin=.15, camera=None, distortion_mode="auto", max_error_px=5.):
        if min(length,width,ppm,margin) <= 0:
            raise ValueError("Measured white-line dimensions and scale required")
        self.length, self.width, self.ppm = length, width, ppm
        h,w = image.shape[:2]
        green = green_quad(image)
        samples = white_samples(image, green)
        self.source,self.k,self.d,self.homography,self.sample_errors=select_geometry(
            samples,w,h,length*ppm,width*ppm,camera,distortion_mode,max_error_px)
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
        self.boundary_errors=verify_rectified_boundary(self.warp(image),self,max_error_px)

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
    """Measure dominant colored mouth in an exterior strip, not a full bbox."""
    hls=cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
    masks={"yellow":mask_hls(hls,(12,100,105),(40,255,255)),
           "blue":mask_hls(hls,(98,45,65),(125,210,255))}
    output={}
    ppm=field.ppm
    for side in (-1,1):
        x0=int(field.pixel((side*field.length/2,0))[0])
        xa,xb=sorted((x0+side*int(.006*ppm),x0+side*int(.125*ppm)))
        xa,xb=max(0,xa),min(image.shape[1],xb)
        if xb-xa<5:
            continue
        ya=max(0,int(field.pixel((0,field.width*.36))[1]))
        yb=min(image.shape[0],int(field.pixel((0,-field.width*.36))[1]))
        candidates=[]
        for color,mask in masks.items():
            strip=mask[ya:yb,xa:xb]
            if strip.size==0:
                continue
            profile=np.count_nonzero(strip,axis=1).astype(np.float32)
            profile=cv2.GaussianBlur(profile.reshape(-1,1),(1,5),0).ravel()
            peak=float(profile.max())
            if peak<max(4,.15*(xb-xa)):
                continue
            rows=np.flatnonzero(profile>=max(4,.45*peak))
            for run in np.split(rows,np.where(np.diff(rows)>1)[0]+1):
                if not .18*ppm<=len(run)<=.55*ppm:
                    continue
                lo=field.metric((x0,ya+run[-1]))[1]
                hi=field.metric((x0,ya+run[0]))[1]
                score=float(np.sum(profile[run]))
                candidates.append((score,color,lo,hi))
        if candidates:
            _,color,lo,hi=max(candidates)
            output[side]=(color,float(lo),float(hi))
    if len(output)==2:
        a,b=output[-1],output[1]
        # Both goals have the same nominal construction; reject, don't invent a length.
        if max(a[2]-a[1],b[2]-b[1])>1.20*min(a[2]-a[1],b[2]-b[1]):
            raise ValueError("Goal mouth lengths disagree: "+str(output))
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


def body_end_led_candidates(image,raw_mask,body,center,p,axis,aspect,ppm,prior,blue_min,debug):
    """Sample in chassis coordinates rather than requiring an isolated contour.

    Search both ends of the long axis (also the short axis for near-square
    silhouettes). A bright transverse strip must have finite endpoints, dark
    chassis immediately inward, and sufficient evidence on the current image.
    The prior only proposes search directions, never creates measurements.
    """
    axes=[axis,-axis]
    if aspect<1.30:
        perpendicular=np.array([-axis[1],axis[0]])
        axes.extend([perpendicular,-perpendicular])
    if prior is not None:
        a=prior.heading
        axes.append(np.array([math.cos(a),-math.sin(a)]))
    candidates=[]
    bright=raw_mask.astype(np.float32)/255.
    dark=body.astype(np.float32)/255.
    def sample(src,normal,tangent,depth,lateral):
        xx=center[0]+normal[0]*depth+tangent[0]*lateral
        yy=center[1]+normal[1]*depth+tangent[1]*lateral
        return cv2.remap(src,xx.astype(np.float32),yy.astype(np.float32),cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    for base in axes:
        for offset in (-12.,0.,12.):
            a=math.atan2(base[1],base[0])+math.radians(offset)
            normal=np.array([math.cos(a),math.sin(a)])
            tangent=np.array([-normal[1],normal[0]])
            axial=(p-center)@normal
            lateral=(p-center)@tangent
            rear=float(np.percentile(axial,2))
            half=float((np.percentile(lateral,98)-np.percentile(lateral,2))/2)
            if not .025*ppm<half<.15*ppm:
                continue
            # Body-relative patch, extending beyond the expected bar ends.
            xs=np.arange(math.floor(-half-.035*ppm),math.ceil(half+.035*ppm)+1,dtype=float)
            ys=np.arange(math.floor(rear-.028*ppm),math.ceil(rear+.022*ppm)+1,dtype=float)
            X,Y=np.meshgrid(xs,ys)
            patch=sample(bright,normal,tangent,Y,X)
            inner=abs(xs)<=.68*half
            if inner.sum()<5:
                continue
            profile=np.mean(patch[:,inner],axis=1)
            rows=np.flatnonzero(profile>=.60)
            runs=[r for r in np.split(rows,np.where(np.diff(rows)>1)[0]+1) if len(r)]
            for run in runs:
                if not max(2,.004*ppm)<=len(run)<=.038*ppm:
                    debug["end_thickness"]=debug.get("end_thickness",0)+1
                    continue
                # Detect the real lateral extent on the selected strip.
                coverage=np.mean(patch[run],axis=0)
                cols=np.flatnonzero(coverage>=.55)
                segments=[r for r in np.split(cols,np.where(np.diff(cols)>1)[0]+1) if len(r)]
                segments=[r for r in segments if abs(float(np.mean(xs[r])))<.026*ppm]
                if not segments:
                    debug["end_span"]=debug.get("end_span",0)+1
                    continue
                seg=max(segments,key=len)
                long=len(seg); short=len(run)
                if not .028*ppm<=long<=min(.135*ppm,1.85*half) or long/short<1.7:
                    debug["end_span"]=debug.get("end_span",0)+1
                    continue
                # A tape crossing the patch usually continues past the body
                # on both ends; an actual LED has two finite endpoints.
                if seg[0]<=1 or seg[-1]>=len(xs)-2:
                    debug["end_tape"]=debug.get("end_tape",0)+1
                    continue
                d=float(np.mean(ys[run])); l=float(np.mean(xs[seg]))
                if abs(d-rear)>.024*ppm:
                    debug["end_offset"]=debug.get("end_offset",0)+1
                    continue
                probe_x,probe_y=np.meshgrid(xs[seg],np.arange(ys[run[-1]]+2,ys[run[-1]]+max(4,int(.020*ppm))))
                inward=float(np.mean(sample(dark,normal,tangent,probe_y,probe_x)))
                if inward<.28:
                    debug["end_body"]=debug.get("end_body",0)+1
                    continue
                occupancy=float(np.mean(patch[np.ix_(run,seg)]))
                if occupancy<.62:
                    debug["end_fill"]=debug.get("end_fill",0)+1
                    continue
                led=center+d*normal+l*tangent
                # Reject unrelated line fragments that extend on both sides in
                # the original ROI, not merely inside the clipped patch.
                ext=[]
                for sign in (-1,1):
                    test=led+sign*tangent*(long/2+np.array([.006,.014,.024])*ppm)[:,None]
                    xy=np.rint(test).astype(int)
                    valid=(xy[:,0]>=0)&(xy[:,0]<raw_mask.shape[1])&(xy[:,1]>=0)&(xy[:,1]<raw_mask.shape[0])
                    ext.append(bool(valid.all() and np.mean(raw_mask[xy[:,1],xy[:,0]]>0)>.65))
                if all(ext):
                    debug["end_tape"]=debug.get("end_tape",0)+1
                    continue
                corners=np.array([led+u*normal*short*.35+v*tangent*long*.35
                                  for u,v in ((-1,-1),(1,-1),(1,1),(-1,1))])
                region=np.zeros(raw_mask.shape,np.uint8)
                cv2.fillConvexPoly(region,np.rint(corners).astype(np.int32),255)
                color=classify_led_pixels(image[region>0],blue_min)
                score=.40*occupancy+.30*min(inward/.65,1.)+.20*min(long/(1.35*half),1.)+.10*(1-min(abs(d-rear)/(.024*ppm),1.))
                if prior is not None:
                    angle=math.atan2(-normal[1],normal[0])
                    score+=.06*max(0.,1-abs(wrap_angle(angle-prior.heading))/math.radians(40))
                if score>=.68:
                    candidates.append((min(score,1.),color,normal,led,(short/ppm,long/ppm)))
                else:
                    debug["end_score"]=debug.get("end_score",0)+1
    return candidates


def robot_observations(image,field,robot_blobs,blue_led_min_light=125,tracks=()):
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
        pad=max(8,int(.045*ppm))
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
        position=field.metric(center+np.array([x0,y0]))
        nearby_tracks=[tr for tr in tracks if tr.visible and tr.heading is not None and tr.position is not None and np.linalg.norm(tr.position-position)<.08]
        prior=min(nearby_tracks,key=lambda tr:np.linalg.norm(tr.position-position)) if nearby_tracks else None
        # Same geometry search for white/off, yellow, and blue.
        raw_mask=np.where((a[:,:,1]>=105)|((rgb.max(axis=2)>=155)&(a[:,:,1]>=75)),255,0).astype(np.uint8)
        # Keep only near the detected chassis; this clips unrelated floor lines.
        chassis=np.zeros(a.shape[:2],np.uint8)
        cv2.drawContours(chassis,[c],-1,255,-1)
        radius=max(3,int(.025*ppm))
        support=cv2.dilate(chassis,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1)))
        mask=cv2.morphologyEx(raw_mask & support,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        candidates=[]
        rejected={}
        def reject(reason):
            rejected[reason]=rejected.get(reason,0)+1
        # Also search a local window around the last measured LED relative to
        # the current chassis, so tape joined to the LED in the global mask
        # cannot remove the short rear bar from the candidate set.
        candidate_masks=[mask]
        if prior is not None and prior.led_offset is not None and prior.led_size is not None:
            angle=prior.heading
            forward=np.array([math.cos(angle),-math.sin(angle)])
            across=np.array([-forward[1],forward[0]])
            off=prior.led_offset
            predicted=center+ppm*(off[0]*forward+off[1]*across)
            half_short,half_long=np.asarray(prior.led_size)*ppm/2+np.array([.010,.010])*ppm
            window=np.array([predicted+u*forward*half_short+v*across*half_long
                             for u,v in ((-1,-1),(1,-1),(1,1),(-1,1))])
            local=np.zeros_like(mask)
            cv2.fillConvexPoly(local,np.rint(window).astype(np.int32),255)
            candidate_masks.append(mask & local)
        all_contours=[cc for cm in candidate_masks for cc in contours(cm)]
        for cc in all_contours:
            ar=cv2.contourArea(cc)
            xx,yy,ww,hh=cv2.boundingRect(cc)
            if xx<=0 or yy<=0 or xx+ww>=a.shape[1] or yy+hh>=a.shape[0]:
                reject("roi_boundary")
                continue
            if not .000055*ppm**2<=ar<=.003*ppm**2:
                reject("area")
                continue
            rr=cv2.minAreaRect(cc)
            short,long=sorted(rr[1])
            if not .034*ppm<=long<=.13*ppm or short<.006*ppm or not 2.0<=long/short<=8.0:
                reject("dimensions")
                continue
            fill=ar/max(short*long,1.)
            if fill<.55:
                reject("fill")
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
                reject("distance")
                continue
            if np.dot(normal,delta)<0:
                normal=-normal
            radial=float(normal@delta/distance)
            # LED long side should span the rear, not run along the chassis.
            if radial<.78 or (aspect>1.25 and abs(normal@axis)<.72):
                reject("axis")
                continue
            # Check the LED lies near the rear edge of this very body, not a
            # separate piece of white tape or a nearby goal.
            extent=(p-center)@normal
            rear=-float(extent.min())
            longitudinal=float(normal@delta)
            lateral=abs(float(major@delta))
            if abs(longitudinal-rear)>.032*ppm or lateral>.045*ppm:
                reject("end_location")
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
                reject("tape")
                continue
            # Check that the inward edge is adjacent to dark body pixels.
            probe=led+normal*(short/2+.010*ppm)
            px,py=np.rint(probe).astype(int)
            if not (0<=px<a.shape[1] and 0<=py<a.shape[0]):
                reject("body_probe")
                continue
            rad=max(2,int(.012*ppm))
            nearby=body[max(0,py-rad):py+rad+1,max(0,px-rad):px+rad+1]
            if not nearby.size or np.mean(nearby>0)<.18:
                reject("body_adjacency")
                continue
            score=.36*radial+.24*fill+.22*min(long/(.065*ppm),1.)+.18*(1-min(abs(longitudinal-rear)/(.032*ppm),1.))
            # Sample the inner bar, excluding edge reflections and background.
            roi_mask=np.zeros(a.shape[:2],np.uint8)
            interior=led+(bb-led)*np.array(0.70)
            cv2.fillConvexPoly(roi_mask,np.rint(interior).astype(np.int32),255)
            color=classify_led_pixels(rgb[roi_mask>0],blue_led_min_light)
            angle=math.atan2(-normal[1],normal[0])
            if prior is not None:
                score+=.08*max(0.,1-abs(wrap_angle(angle-prior.heading))/math.radians(40))
            candidates.append((min(score,1.),color,normal,led,(short/ppm,long/ppm)))
        def ambiguous(items):
            if not items:
                return True
            best=max(items,key=lambda z:z[0])
            return best[0]<.62 or any(q>best[0]-.07 and np.dot(best[2],n)<.85 for q,_,n,_,_size in items)
        method="contour"
        contour_count=len(candidates)
        if ambiguous(candidates):
            fallback=body_end_led_candidates(rgb,raw_mask,body,center,p,axis,aspect,ppm,prior,blue_led_min_light,rejected)
            if fallback:
                # Do not combine conflicting modes into a false confident result.
                candidates=fallback
                method="body_end"
        candidates.sort(key=lambda z:z[0],reverse=True)
        heading=None
        team="unknown"
        quality=0.
        led_offset=None
        led_size=None
        length_m=max(rect[1])/ppm
        width_m=min(rect[1])/ppm
        if candidates:
            quality,color,normal,led,led_size=candidates[0]
            # Do not manufacture a direction if two plausible bars disagree.
            conflict=any(s>quality-.07 and (np.dot(normal,n)<.85)
                         for s,col,n,_,_size in candidates[1:])
            if quality>=.62 and not conflict:
                heading=math.atan2(-normal[1],normal[0])
                team=color if color in ("yellow","blue") else "unknown"
                side=np.array([-normal[1],normal[0]])
                led_offset=np.array([(led-center)@normal,(led-center)@side])/ppm
                aa=p@normal
                bb=p@side
                lo,hi=np.percentile(aa,[1,99])
                blo,bhi=np.percentile(bb,[1,99])
                # Keep body center independent of LED detection success.
                length_m=(hi-lo)/ppm
                width_m=(bhi-blo)/ppm
            else:
                reject("ambiguous_or_low_score")
                quality=0.
        global_center=center+np.array([x0,y0])
        result.append({"p":field.metric(global_center),
                       "bbox":np.array([x,y,w,h],float)/ppm,
                       "heading":heading,"team":team,"quality":float(quality),
                       "length":float(length_m),"width":float(width_m),
                       "led_offset":led_offset,"led_size":led_size if heading is not None else None,
                       "led_candidates":len(candidates),"led_method":method if heading is not None else "missing",
                       "led_rejected":rejected,"contour_candidates":contour_count,
                       "led_center_px":(led+np.array([x0,y0])).tolist() if heading is not None else None})
    return result


class ConstantVelocityFilter:
    """Small linear Kalman filter; independent position/velocity axes."""
    def __init__(self,z,sigma,accel):
        z=np.atleast_1d(np.asarray(z,float))
        self.n=len(z)
        self.x=np.r_[z,np.zeros(self.n)]
        self.P=np.diag(np.r_[np.full(self.n,sigma**2),np.full(self.n,1.)])
        self.sigma=sigma
        self.accel=accel

    def step(self,z,dt):
        n=self.n
        I=np.eye(n)
        F=np.block([[I,dt*I],[np.zeros((n,n)),I]])
        G=np.vstack([.5*dt**2*I,dt*I])
        self.x=F@self.x
        self.P=F@self.P@F.T+G@G.T*self.accel**2
        if z is not None:
            H=np.hstack([I,np.zeros((n,n))])
            R=I*self.sigma**2
            S=H@self.P@H.T+R
            K=np.linalg.solve(S,H@self.P).T
            self.x+=K@(np.atleast_1d(z)-H@self.x)
            A=np.eye(2*n)-K@H
            self.P=A@self.P@A.T+K@R@K.T
        return self.x[:n].copy(),self.x[n:].copy()


class RobotTrack(Track):
    def __init__(self,ident):
        super().__init__(ident)
        self.team="unknown"
        self.identity_team="unknown"
        self.color_candidate="unknown"
        self.color_count=0
        self.heading=None
        self.heading_confidence=0.
        self.heading_measured=False
        self.heading_stamp=None
        self.heading_filter=None
        self.position_filter=None
        self.led_offset=None
        self.led_size=None
        self.led_candidates=0
        self.led_method="missing"
        self.angular_velocity=None
        self.bbox=None
        self.length=.16
        self.width=.12
        self.pending=None
        self.pending_count=0
        self.reacquired=False
        self.match_confidence=0.

    def observe(self,obs,t,continuous,confidence):
        dt=t-self.stamp if self.stamp is not None else 0.
        had_position=self.position is not None
        continuous=bool(continuous and .001<dt<.30)
        if not continuous:
            self.visible=False
            self.position_filter=None
            self.heading_filter=None
            self.heading_stamp=None
            self.heading=None
            self.led_offset=None
            self.led_size=None
        super().update(obs["p"],t)
        if self.position_filter is None:
            self.position_filter=ConstantVelocityFilter(obs["p"],.003,2.5)
            self.velocity_valid=False
        else:
            self.position,self.velocity=self.position_filter.step(obs["p"],dt)
        old_measured=self.heading_measured and continuous
        self.heading_measured=False
        self.angular_velocity=None
        measurement=obs["heading"]
        # Reject sudden implausible flips during continuous tracking. If the
        # direction truly changed, the hold expires and a new measurement starts.
        if measurement is not None and self.heading is not None and continuous:
            if abs(wrap_angle(measurement-self.heading))>math.radians(65):
                measurement=None
        if measurement is not None:
            if self.heading_filter is None:
                self.heading_filter=ConstantVelocityFilter([measurement],math.radians(2.5),12.)
                self.heading=wrap_angle(measurement)
            else:
                z=self.heading_filter.x[0]+wrap_angle(measurement-self.heading_filter.x[0])
                angle,omega=self.heading_filter.step([z],dt)
                self.heading=wrap_angle(float(angle[0]))
                if old_measured and self.velocity_valid:
                    self.angular_velocity=float(omega[0])
            self.heading_stamp=t
            self.heading_measured=True
            self.heading_confidence=obs["quality"]
            self.led_offset=obs.get("led_offset")
            self.led_size=obs.get("led_size")
        elif continuous and self.heading_filter is not None and self.heading_stamp is not None and t-self.heading_stamp<=.25:
            angle,_=self.heading_filter.step(None,dt)
            self.heading=wrap_angle(float(angle[0]))
            self.heading_confidence*=.85
        else:
            self.heading=None
            self.heading_filter=None
            self.heading_stamp=None
            self.heading_confidence=0.
            self.led_offset=None
            self.led_size=None
        self.led_candidates=obs.get("led_candidates",0)
        self.led_method=obs.get("led_method","missing") if self.heading_measured else "missing"
        observed=obs["team"] if self.heading_measured else "unknown"
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
        if self.heading_measured or self.heading is None:
            self.length=obs["length"]
            self.width=obs["width"]
        self.reacquired=bool(had_position and not continuous)
        self.match_confidence=confidence
        self.pending=None
        self.pending_count=0

    def miss(self):
        super().miss()
        self.heading=None
        self.heading_measured=False
        self.heading_stamp=None
        self.heading_filter=None
        self.position_filter=None
        self.led_offset=None
        self.led_size=None
        self.led_method="missing"
        self.led_candidates=0
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
