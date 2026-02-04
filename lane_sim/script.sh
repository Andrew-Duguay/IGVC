#Run from lane_sim

#source the models in lane_sim
export GZ_SIM_RESOURCE_PATH=$HOME/gazebo-sim/lane_sim

#Runs the simulation
gz sim ./world.sdf 

# Runs the bridge so that sensor data from gazebo is published to ros topic
### ros2 run ros_gz_bridge parameter_bridge   /camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image   /camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image   /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist