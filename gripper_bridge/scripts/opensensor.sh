#!/bin/bash

sudo -S sh /home/de/catkin_ws/src/hirop_ros/gripper_bridge/scripts/opensensor_order.sh << EOF
0
EOF

wait
exit 0


