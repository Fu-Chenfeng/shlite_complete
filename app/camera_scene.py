"""Visual-only camera cones shared by RViz and Gazebo."""
import math
import xml.etree.ElementTree as ET


LENGTH = 0.12
RADIUS = 0.04
ALPHA = 0.3
MESH_URI = 'file:///project/assets/models/camera_view_cone/cone.stl'
MARKER_NAMESPACE = 'camera_views'


def cone_triangles(segments=48):
    """Return an outward-facing closed unit cone: tip at zero, base at +Z."""
    if type(segments) is not int or segments < 3:
        raise ValueError('A cone requires at least three circular segments')
    ring = [(math.cos(2*math.pi*i/segments), math.sin(2*math.pi*i/segments), 1.0)
            for i in range(segments)]
    triangles = []
    for index, current in enumerate(ring):
        following = ring[(index+1) % segments]
        triangles.append(((0.0, 0.0, 0.0), following, current))
        triangles.append(((0.0, 0.0, 1.0), current, following))
    return triangles


def cone_model_name(view):
    return 'camera_view_cone_%d' % view['index']


def view_markers(geometry):
    """Place each cone tip at its optical center, opening along optical +Z."""
    import numpy as np
    from tf.transformations import quaternion_from_matrix
    from visualization_msgs.msg import Marker, MarkerArray

    result = MarkerArray()
    for view in geometry['views']:
        camera = np.asarray(view['camera_optical_to_world'], dtype=float)
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = MARKER_NAMESPACE
        marker.id = view['index']
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = MESH_URI
        marker.mesh_use_embedded_materials = False
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, camera[:3, 3])
        quaternion = quaternion_from_matrix(camera)
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = map(float, quaternion)
        marker.scale.x, marker.scale.y, marker.scale.z = RADIUS, RADIUS, LENGTH
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, ALPHA
        result.markers.append(marker)
    return result


def clear_markers():
    """Clear the dedicated camera-view marker topic."""
    from visualization_msgs.msg import Marker, MarkerArray
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.ns = MARKER_NAMESPACE
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def cone_sdf(view):
    """Build a local cone visual; SpawnModel supplies the camera world pose."""
    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=cone_model_name(view))
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0 0 0 0 0 0'
    link = ET.SubElement(model, 'link', name='cone')
    ET.SubElement(link, 'gravity').text = 'false'
    visual = ET.SubElement(link, 'visual', name='view_direction')
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    ET.SubElement(visual, 'transparency').text = str(1.0-ALPHA)
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = MESH_URI
    ET.SubElement(mesh, 'scale').text = '%g %g %g' % (RADIUS, RADIUS, LENGTH)
    material = ET.SubElement(visual, 'material')
    for component in ('ambient', 'diffuse', 'emissive'):
        ET.SubElement(material, component).text = '1 1 0 1'
    return ET.tostring(sdf, encoding='unicode')
