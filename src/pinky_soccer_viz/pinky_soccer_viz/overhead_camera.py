"""MuJoCo overhead RGB camera; no pose/object metadata enters the referee."""
import math


class OverheadCamera:
    def __init__(self, node, model):
        import mujoco
        from sensor_msgs.msg import Image, CameraInfo
        self.node, self.model, self.Image = node, model, Image
        self.width, self.height = 1280, 720
        self.last = -math.inf
        self.renderer = None
        self.cutaway = node.declare_parameter('overhead_cutaway_goals', True).value
        self.calibrated = False
        from pinky_soccer_referee.msg import RefereeState
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.state_sub = node.create_subscription(RefereeState, '/referee/state',
            lambda m: setattr(self, 'calibrated', m.calibrated),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.roofs = {i for i in range(model.ngeom) if
            (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').endswith('_goal_roof')}
        self.image = node.create_publisher(Image, '/referee_camera/image_raw', 1)
        self.info_pub = node.create_publisher(CameraInfo, '/referee_camera/camera_info', 1)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = [0., 0., 0.]
        self.camera.azimuth, self.camera.elevation, self.camera.distance = 90., -90., 2.0
        f = self.height/(2*math.tan(math.radians(model.vis.global_.fovy)/2))
        self.info = CameraInfo(width=self.width, height=self.height,
            distortion_model='plumb_bob', d=[0.]*5,
            k=[f,0.,self.width/2,0.,f,self.height/2,0.,0.,1.],
            r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            p=[f,0.,self.width/2,0.,0.,f,self.height/2,0.,0.,0.,1.,0.])
        self.info.header.frame_id = 'referee_camera_optical_frame'

    def publish(self, data, stamp):
        import mujoco
        if not self.image.get_subscription_count() or data.time-self.last < .1-1e-8:
            return
        self.last = data.time
        if self.renderer is None:
            # Offscreen buffer sizing is rendering-only; named robot cameras retain
            # their own FOV, pose and output dimensions.
            self.model.vis.global_.offwidth = max(self.width, self.model.vis.global_.offwidth)
            self.model.vis.global_.offheight = max(self.height, self.model.vis.global_.offheight)
            self.renderer = mujoco.Renderer(self.model, self.height, self.width)
            self.option = mujoco.MjvOption()
            for group in (1, 3, 5):
                self.option.geomgroup[group] = 0
        self.renderer.update_scene(data, camera=self.camera, scene_option=self.option)
        if self.cutaway and self.calibrated:
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            # Top-down scoring needs to see the ball after it crosses the mouth.
            # Hide only roof pixels in this camera's scene, never model/physics or
            # the robot cameras. This is an explicit simulator cutaway view.
            for geom in self.renderer.scene.geoms[:self.renderer.scene.ngeom]:
                if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and geom.objid in self.roofs:
                    geom.rgba[3] = 0.
        rgb = self.renderer.render()
        msg = self.Image()
        msg.header.stamp, msg.header.frame_id = stamp, self.info.header.frame_id
        msg.width, msg.height = self.width, self.height
        msg.encoding, msg.step = 'rgb8', self.width*3
        msg.data = rgb.tobytes()
        self.image.publish(msg)
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)
