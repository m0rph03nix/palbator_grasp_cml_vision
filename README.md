

docker run -it --device=/dev/video2 --device=/dev/video3 --device=/dev/video4 --device=/dev/video5 --device=/dev/video6 --device=/dev/video7  --network=host -e ROS_MASTER_URI=http://10.68.0.1:11311 -e ROS_IP=10.68.0.128 --rm --name palbator_grasp_cml_vision palbator_grasp_cml_vision