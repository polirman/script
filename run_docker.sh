docker run -itd --privileged --net=host --ipc=host --pid=host  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /etc/localtime:/etc/localtime:ro --name all registry.cn-shanghai.aliyuncs.com/ivedu/bit_humble_msgs:latest bash


docker run -it --name qrq \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="QT_XKB_CONFIG_ROOT=/usr/share/X11/xkb" \
  --env="ROS_DOMAIN_ID=0" \
  --env="ROS_IP=192.168.1.100" \
  --network=host \
  -v /home/polirman/project/qrq_sanweiguihua:/root/ws \
  runqiqiu/pnc:2.6_x86 bash
  
 docker run -itd --net=host --ipc=host  -e DISPLAY=$DISPLAY -v /home/polirman/project/qrq_sanweiguihua:/root/ws --name qrq runqiqiu/pnc:2.6_x86 bash

