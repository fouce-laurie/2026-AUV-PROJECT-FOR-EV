import re

with open('/home/origin/AUV2026/src/stonefish_ros2/Data/xunyun.scn', 'r') as f:
    text = f.read()

# Fix odometry, imu, pressure back to rpy="0.0 0.0 0.0"
def fix_sensor_rpy(sensor_name, text):
    pattern = r'(<sensor name="' + sensor_name + r'".*?<origin\s+)rpy="[^"]+"(\s+xyz="[^"]+")'
    return re.sub(pattern, r'\1rpy="0.0 0.0 0.0"\2', text, flags=re.DOTALL)

text = fix_sensor_rpy("dynamics", text)
text = fix_sensor_rpy("pressure", text)
text = fix_sensor_rpy("imu_filter", text)

with open('/home/origin/AUV2026/src/stonefish_ros2/Data/xunyun.scn', 'w') as f:
    f.write(text)

print("Sensors Odom, Pressure, IMU reset to 0.0 0.0 0.0")

with open('/home/origin/AUV2026/src/stonefish_ros2/Data/xunyun.scn', 'r') as f:
    text = f.read()

pattern = r'(<comm name="USBL".*?<origin) rpy="[^"]+"(\s+xyz="[^"]+")'
text = re.sub(pattern, r'\1 rpy="0.0 0.0 0.0"\2', text, flags=re.DOTALL)

with open('/home/origin/AUV2026/src/stonefish_ros2/Data/xunyun.scn', 'w') as f:
    f.write(text)
