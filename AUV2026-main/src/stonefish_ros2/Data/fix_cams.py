import re

with open('/home/origin/AUV2026/src/stonefish_ros2/Data/xunyun.scn', 'r') as f:
    text = f.read()

# Change front cams to exactly 1.5708 0 1.5708
text = re.sub(r'name="front_cam_left"[\s\S]*?origin rpy="[^"]+"', 
              r'name="front_cam_left" type="camera" rate="10.0">\n\t\t\t<link name="Vehicle"/>\n\t\t\t<origin rpy="1.5708 0 1.5708"', text)
text = re.sub(r'name="front_cam_right"[\s\S]*?origin rpy="[^"]+"', 
              r'name="front_cam_right" type="camera" rate="10.0">\n\t\t\t<link name="Vehicle"/>\n\t\t\t<origin rpy="1.5708 0 1.5708"', text)

# Change down cams to what they need to point down properly, perhaps original was 0 0 -1.5708, now let's set to 0 0 1.5708
text = re.sub(r'name="down_cam_left"[\s\S]*?origin rpy="[^"]+"', 
              r'name="down_cam_left" type="camera" rate="30.0">\n\t\t\t<link name="Vehicle"/>\n\t\t\t<origin rpy="0.0 0.0 1.5708"', text)
text = re.sub(r'name="down_cam_right"[\s\S]*?origin rpy="[^"]+"', 
              r'name="down_cam_right" type="camera" rate="30.0">\n\t\t\t<link name="Vehicle"/>\n\t\t\t<origin rpy="0.0 0.0 1.5708"', text)

# Just run a quick print
print("Fixed cameras.")
with open('/home/origin/AUV2026/src/stonefish_ros2/Data/xunyun.scn', 'w') as f:
    f.write(text)
