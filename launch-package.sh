name=$1
source ./install/setup.bash
ros2 launch ${name}_pkg ${name}_pkg_launch_file.launch.py