#!/usr/bin/env python3
"""Visual-only LED adapters using the unchanged physical set_led service contract."""
import xml.etree.ElementTree as ET

def gazebo_lamp_materials(xml):
    """Explicit material on the existing mesh enables runtime color overrides."""
    root = ET.fromstring(xml)
    for visual in root.iter('visual'):
        if visual.get('name', '').endswith('robot_lamp_visual'):
            old = visual.find('material')
            if old is not None:
                visual.remove(old)
            material = ET.SubElement(visual, 'material')
            for channel in ('ambient', 'diffuse', 'specular', 'emissive'):
                ET.SubElement(material, channel).text = '0 0 0 1'
    return ET.tostring(root, encoding='unicode')


def led_materials(scene):
    """Assign dedicated emissive materials to existing lamp visuals; no geoms added."""
    tree = ET.parse(scene)
    root = tree.getroot()
    asset = root.find('asset')
    for geom in root.iter('geom'):
        if 'robot_lamp_visual' not in geom.get('name', ''):
            continue
        material = geom.get('name')+'_soccer_led'
        if asset.find(f"material[@name='{material}']") is None:
            ET.SubElement(asset, 'material', name=material, rgba='0 0 0 1', emission='1')
        geom.set('material', material)
        # An explicit geom rgba overrides material rgba in MuJoCo; use its default.
        geom.attrib.pop('rgba', None)
    tree.write(scene, encoding='unicode')
    return scene


def install_mujoco_led(bridge, opponent=''):
    """Reuse official bridge lifecycle; mutate materials only on service callbacks."""
    base = bridge.Bridge
    from pinky_interfaces.srv import SetLed

    class LedBridge(base):
        def __init__(self, physics):
            super().__init__(physics)
            import mujoco
            model = physics.model
            def callback(prefix):
                ids = [i for i in range(model.ngeom)
                       if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith(
                           prefix+'robot_lamp_visual')]
                def receive(req, res):
                    if req.command not in ('fill', 'clear') or not all(0 <= x <= 255 for x in (req.r, req.g, req.b)):
                        res.success, res.message = False, 'Expected fill/clear, RGB0..255'
                        return res
                    rgba = [v/255. for v in (req.r, req.g, req.b)]+[1.]
                    if req.command == 'clear':
                        rgba = [0., 0., 0., 1.]
                    for i in ids:
                        mat = model.geom_matid[i]
                        if mat >= 0:
                            model.mat_rgba[mat] = rgba
                        else:
                            model.geom_rgba[i] = rgba
                    res.success, res.message = bool(ids), 'Existing lamp visual materials updated'
                    return res
                return receive
            self.create_service(SetLed, 'set_led', callback(''))
            if opponent:
                self.create_service(SetLed, '/'+opponent+'/set_led', callback('opp_'))
    bridge.Bridge = LedBridge


def main():
    """Gazebo counterpart: material-only Visual command for the existing lamp."""
    import rclpy
    from rclpy.node import Node
    from pinky_interfaces.srv import SetLed
    from gz.transport13 import Node as GzNode
    from gz.msgs10.visual_pb2 import Visual
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.boolean_pb2 import Boolean
    rclpy.init()
    node = Node('soccer_led_visual')
    world = node.declare_parameter('world_name', 'hardboard_soccer').value
    model = node.declare_parameter('model_name', 'pinky').value
    transport = GzNode()
    visual_id = [None]
    def callback(req, res):
        if req.command not in ('fill', 'clear') or not all(0 <= v <= 255 for v in (req.r, req.g, req.b)):
            res.success, res.message = False, 'Expected fill/clear RGB0..255'
            return res
        if visual_id[0] is None:
            ok, scene = transport.request('/world/'+world+'/scene/info', Empty(), Empty, Scene, 100)
            if ok:
                for robot in scene.model:
                    if robot.name.split('::')[-1] != model:
                        continue
                    for link in robot.link:
                        for visual in link.visual:
                            if visual.name.split('::')[-1] == 'robot_lamp_visual':
                                visual_id[0] = visual.id
            if visual_id[0] is None:
                res.success, res.message = False, 'Lamp visual not available yet'
                return res
        msg = Visual()
        msg.id = visual_id[0]
        values = (req.r, req.g, req.b) if req.command == 'fill' else (0, 0, 0)
        for key in ('ambient', 'diffuse', 'emissive'):
            color = getattr(msg.material, key)
            color.r, color.g, color.b, color.a = *[v/255. for v in values], 1.
        ok, reply = transport.request('/world/'+world+'/visual_config', msg, Visual, Boolean, 100)
        res.success = bool(ok and reply.data)
        res.message = 'Existing lamp visual material update '+('accepted' if res.success else 'failed')
        return res
    node.create_service(SetLed, 'set_led', callback)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
