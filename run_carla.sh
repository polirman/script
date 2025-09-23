 cd /home/polirman/Downloads/CARLA_0.9.14
 ./CarlaUE4.sh -quality-level=Low 
 ./CarlaUE4.sh -RenderOffScreen -quality-level=Low 
 
 
export CARLA_ROOT=/home/polirman/Downloads/CARLA_0.9.14
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.14-py3.10-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
cd /home/polirman/carla_ros_bridge/
source install/setup.bash 

 ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
 
 
export ROS_DOMAIN_ID=0
export ROS_IP=192.168.43.32

echo $ROS_DOMAIN_ID
echo $ROS_IP



conda activate carla_py37
python3 ~/Downloads/CARLA_0.9.15/PythonAPI/examples/1_manual_control_carsim.py 
conda deactivate
