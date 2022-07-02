# bag2_to_image

convert sensor_msgs::msg::Image or sensor_msgs::msg::CompressedImage in rosbag2 to .jpeg files.

## Usage

```bash
mkdir -p <your workspace>/src
cd <your workspace>/src
git clone https://github.com/wep21/bag2_to_image.git
cd ../../
colcon build
source install/setup.bash
ros2 run bag2_to_image bag2_to_image_exe --ros-args -p uri:=<your rosbag2 file> -p image_topic:=<image_topic_name> -p image_save_directory:=<your directory to save images>
```