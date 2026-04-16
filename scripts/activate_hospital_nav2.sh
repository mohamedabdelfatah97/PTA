#!/bin/bash
# Hospital Nav2 Activation Script
# Robot spawn: x=0.0, y=-5.0, yaw=1.5707963 (facing north)

echo "=== Waiting for map_server ==="
until ros2 service list 2>/dev/null | grep -q "map_server/change_state"; do
    sleep 3
done
echo "map_server ready — activating localization..."

ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /amcl/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /amcl/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

echo "=== Waiting 10s for map frame ==="
sleep 10

echo "=== Waiting for controller_server ==="
until ros2 service list 2>/dev/null | grep -q "controller_server/change_state"; do
    sleep 3
done
echo "Navigation nodes ready — configuring..."

ros2 service call /controller_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /smoother_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /planner_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /behavior_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /waypoint_follower/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /velocity_smoother/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"

echo "=== Activating navigation ==="
ros2 service call /controller_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /smoother_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /planner_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /behavior_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /waypoint_follower/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 service call /velocity_smoother/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

echo "=== Hospital Nav2 fully active! Ready for navigation ==="
