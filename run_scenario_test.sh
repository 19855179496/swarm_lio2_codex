#!/bin/bash
# Automated test script for three trajectory scenarios
# Usage: bash run_scenario_test.sh [circle|diagonal|waypoint]

set -e
SCENARIO=${1:-circle}
WORKSPACE="/home/hr/aaworkspace/simulate_test_codex"
RESULT_DIR="$WORKSPACE/scenario_results/$SCENARIO"
mkdir -p "$RESULT_DIR"

# Use bash setup files to avoid zsh cd -q issue
source_marsim() { source "$WORKSPACE/marsim/devel/setup.bash" 2>/dev/null || source "$WORKSPACE/marsim/devel/setup.zsh" 2>/dev/null; }
source_swarm() { source "$WORKSPACE/swarm_lio2_v1/devel/setup.bash" 2>/dev/null || source "$WORKSPACE/swarm_lio2_v1/devel/setup.zsh" 2>/dev/null; }

echo "=========================================="
echo "Testing scenario: $SCENARIO"
echo "Results dir: $RESULT_DIR"
echo "=========================================="

# Cleanup
echo "[1/6] Cleaning up ROS processes..."
pkill -9 -f roslaunch 2>/dev/null || true
pkill -9 -f rosmaster 2>/dev/null || true
pkill -9 -f laserMapping 2>/dev/null || true
pkill -9 -f quadrotor 2>/dev/null || true
pkill -9 -f cascadePID 2>/dev/null || true
pkill -9 -f pcl_render 2>/dev/null || true
pkill -9 -f ros_bridge 2>/dev/null || true
pkill -9 -f rviz 2>/dev/null || true
pkill -9 -f map_pub 2>/dev/null || true
sleep 2

# Launch MARSIM
echo "[2/6] Launching MARSIM ($SCENARIO)..."
source_marsim
roslaunch test_interface triple_drone_mid360_${SCENARIO}.launch wait_for_start_signal:=true &>/dev/null &
MARSIM_PID=$!
sleep 6

# Verify MARSIM
NODE_COUNT=$(rosnode list 2>/dev/null | grep quad | wc -l)
if [ "$NODE_COUNT" -lt 9 ]; then
    echo "ERROR: MARSIM nodes not ready (only $NODE_COUNT quad nodes)"
    kill $MARSIM_PID 2>/dev/null
    exit 1
fi
echo "  MARSIM OK: $NODE_COUNT quad nodes"

# Launch ros_bridge
echo "[3/6] Launching ros_bridge..."
source_swarm
roslaunch udp_bridge ros_bridge.launch log_every_quad:=20 log_every_ge:=10 &>/dev/null &
sleep 2

# Launch Swarm-LIO
echo "[4/6] Launching Swarm-LIO..."
roslaunch swarm_lio simulation_three.launch &>/dev/null &
SWARM_PID=$!
sleep 8

# Verify Swarm-LIO
LIO_NODES=$(rosnode list 2>/dev/null | grep laserMapping | wc -l)
if [ "$LIO_NODES" -lt 3 ]; then
    echo "ERROR: Swarm-LIO nodes not ready (only $LIO_NODES)"
    exit 1
fi

# Check parameters
ROT_THRESH=$(rosparam get /laserMapping_quad0/multiuav/traj_match_max_rot_angle_deg 2>/dev/null)
DIST_THRESH=$(rosparam get /laserMapping_quad0/multiuav/traj_match_current_dist_thresh 2>/dev/null)
echo "  Swarm-LIO OK: $LIO_NODES nodes, rot_thresh=$ROT_THRESH deg, dist_thresh=$DIST_THRESH m"

# Wait for LIO initialization
echo "[5/6] Waiting for LIO initialization..."
sleep 10

# Start trajectory
echo "  Starting trajectory..."
rostopic pub -1 /start_figure8 std_msgs/Bool "data: true" &>/dev/null

# Wait for waypoint start if needed
if [ "$SCENARIO" = "waypoint" ]; then
    sleep 2
    rostopic pub -1 /start_waypoint_mission std_msgs/Bool "data: true" &>/dev/null
fi

# Monitor for 45 seconds
echo "[6/6] Monitoring for 45 seconds..."
sleep 15

# First check: teammate count
echo "--- Check at t=15s ---"
T0=$(timeout 3 rostopic echo /quad0/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
T1=$(timeout 3 rostopic echo /quad1/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
T2=$(timeout 3 rostopic echo /quad2/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
echo "  Teammate count: quad0=$T0, quad1=$T1, quad2=$T2"

# Check extrinsic values
echo "  Extrinsic values:"
timeout 3 rostopic echo /global_extrinsic_to_teammate 2>/dev/null | grep -E "drone_id|teammate_id|rot_deg|trans:" | head -24 | tee $RESULT_DIR/extrinsic_t15s.txt

sleep 15
echo "--- Check at t=30s ---"
T0=$(timeout 3 rostopic echo /quad0/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
T1=$(timeout 3 rostopic echo /quad1/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
T2=$(timeout 3 rostopic echo /quad2/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
echo "  Teammate count: quad0=$T0, quad1=$T1, quad2=$T2"

timeout 3 rostopic echo /global_extrinsic_to_teammate 2>/dev/null | grep -E "drone_id|teammate_id|rot_deg|trans:" | head -24 | tee $RESULT_DIR/extrinsic_t30s.txt

sleep 15
echo "--- Check at t=45s ---"
T0=$(timeout 3 rostopic echo /quad0/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
T1=$(timeout 3 rostopic echo /quad1/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
T2=$(timeout 3 rostopic echo /quad2/connected_teammate_num 2>/dev/null | grep "data:" | tail -1 | awk '{print $2}')
echo "  Teammate count: quad0=$T0, quad1=$T1, quad2=$T2"

timeout 3 rostopic echo /global_extrinsic_to_teammate 2>/dev/null | grep -E "drone_id|teammate_id|rot_deg|trans:" | head -24 | tee $RESULT_DIR/extrinsic_t45s.txt

# Check for temp trackers
TEMP_COUNT=$(timeout 3 rostopic echo /quad0/temp_tracker 2>/dev/null | head -5 | wc -l)
echo "  Temp trackers on quad0: $TEMP_COUNT lines"

echo ""
echo "=========================================="
echo "Scenario $SCENARIO test COMPLETE"
echo "Results saved to: $RESULT_DIR"
echo "=========================================="

# Cleanup
pkill -9 -f roslaunch 2>/dev/null || true
pkill -9 -f rosmaster 2>/dev/null || true
pkill -9 -f laserMapping 2>/dev/null || true
pkill -9 -f quadrotor 2>/dev/null || true
pkill -9 -f cascadePID 2>/dev/null || true
pkill -9 -f pcl_render 2>/dev/null || true
pkill -9 -f ros_bridge 2>/dev/null || true
pkill -9 -f rviz 2>/dev/null || true
pkill -9 -f map_pub 2>/dev/null || true
sleep 2
