import xml.etree.ElementTree
from json import dumps

# adjust this to your SDF file location
filename = '/home/mykyta/catkin_ws/src/drive_gazebo_plugins/worlds/world.sdf'
file = xml.etree.ElementTree.parse(filename)
sdf = file.getroot()
signs_in_scene = {}
for model in sdf.find('world').findall('model'):
    if 'Sign' in model.attrib['name']:
        signs_in_scene[model.attrib['name']] = model.find('link').find('visual').find('material').find('script').find('name').text

sign_label_plugin_element = """
<plugin name="sign_label_plugin" filename="libdrive_sign_label_plugin.so">
    <cameraName>camera</cameraName>
    <alwaysOn>true</alwaysOn>
    <updateRate>1</updateRate>
    <robotNamespace>/</robotNamespace>
    <topicName>camera_raw</topicName>
    <output_folder>/hdd/ground_truth</output_folder>
    <sign_area_threshold>60</sign_area_threshold>
    <signs_in_world>{signs}</signs_in_world>
</plugin>
""".format(signs=dumps(signs_in_scene))

print(sign_label_plugin_element)

for model in sdf.find('world').findall('model'):
    if 'EgoVehicle' in model.attrib['name']:
        print(model.find('link').find('sensor').getchildren())
        model.find('link').find('sensor').append(xml.etree.ElementTree.fromstring(sign_label_plugin_element))
        print(model.find('link').find('sensor').getchildren())

file.write('/home/mykyta/catkin_ws/src/drive_gazebo_plugins/worlds/world_testing.sdf')
