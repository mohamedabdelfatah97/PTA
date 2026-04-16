#!/bin/bash
# Activate Nav2 lifecycle nodes with timeouts
# Never hangs — each call has a 15s timeout

source /opt/ros/humble/setup.bash
source ~/pta_sim_ws/nav2_overlay/install/setup.bash
source ~/pta_sim_ws/install/setup.bash

call_service() {
    local service=$1
    local transition=$2
    local label=$3
    echo ">>> $label ($service)"
    timeout 15 ros2 service call $service \
        lifecycle_msgs/srv/ChangeState \
        "{transition: {id: $transition}}" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "    WARN: timed out or failed — continuing"
    fi
}

echo "=== Waiting for map_server ==="
until ros2 service list 2>/dev/null | grep -q "map_server/change_state"; do
    sleep 2
done
echo "map_server ready — activating localization..."

call_service /map_server/change_state 1 "configure map_server"
call_service /amcl/change_state 1 "configure amcl"
call_service /map_server/change_state 3 "activate map_server"
call_service /amcl/change_state 3 "activate amcl"

echo "=== Waiting 10s for map frame ==="
sleep 10

echo "=== Waiting for controller_server ==="
until ros2 service list 2>/dev/null | grep -q "controller_server/change_state"; do
    sleep 2
done
echo "Navigation nodes ready — configuring..."

call_service /controller_server/change_state 1 "configure controller_server"
call_service /smoother_server/change_state 1 "configure smoother_server"
call_service /planner_server/change_state 1 "configure planner_server"
call_service /behavior_server/change_state 1 "configure behavior_server"
call_service /bt_navigator/change_state 1 "configure bt_navigator"
call_service /waypoint_follower/change_state 1 "configure waypoint_follower"
call_service /velocity_smoother/change_state 1 "configure velocity_smoother"

echo "=== Activating navigation ==="
call_service /controller_server/change_state 3 "activate controller_server"
call_service /smoother_server/change_state 3 "activate smoother_server"
call_service /planner_server/change_state 3 "activate planner_server"
call_service /behavior_server/change_state 3 "activate behavior_server"
call_service /bt_navigator/change_state 3 "activate bt_navigator"
call_service /waypoint_follower/change_state 3 "activate waypoint_follower"
call_service /velocity_smoother/change_state 3 "activate velocity_smoother"

echo "=== Nav2 fully active! Ready for navigation ==="
