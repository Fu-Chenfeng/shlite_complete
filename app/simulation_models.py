"""Keep gazebo2rviz's filesystem model lookup consistent with the loaded world."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def sync_models(world_file, models_directory):
    world = ET.parse(world_file).getroot().find('world')
    for model in world.findall('model'):
        destination = Path(models_directory) / model.get('name')
        destination.mkdir(parents=True, exist_ok=True)
        # ModelStates supplies the world transform separately to the bridge.
        for pose in model.findall('pose'):
            model.remove(pose)
        # gazebo2rviz accepts filesystem paths and model://, but not file://.
        for uri in model.findall('.//mesh/uri'):
            if uri.text.startswith('file://'):
                uri.text = uri.text[len('file://'):]
        sdf = ET.Element('sdf', version='1.6')
        sdf.append(model)
        ET.ElementTree(sdf).write(destination / 'model.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    import rospkg
    directory = Path(rospkg.RosPack().get_path('sr_description_common')) / 'models'
    sync_models(sys.argv[1], directory)
