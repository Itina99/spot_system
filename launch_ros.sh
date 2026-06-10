#!/bin/bash

SESSION="spot_sim"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="/home/spot/spot_sim_ws/install/setup.bash"

LOG_DIR="/home/spot/PycharmProjects/spot_system/logs"

mkdir -p "$LOG_DIR"

RUN_ID=$(date +"%Y%m%d_%H%M%S")
ALGO_LOG="$LOG_DIR/exploration_${RUN_ID}.log"

cleanup() {
    echo ""
    echo "Pulizia processi..."

    tmux kill-session -t "$SESSION" 2>/dev/null

    pkill -f "ign gazebo" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f rviz 2>/dev/null
    pkill -f "ros2 launch" 2>/dev/null

    sleep 2

    echo "Pulizia completata."
}

trap cleanup SIGINT SIGTERM

echo "======================================================"
echo "Pulizia eventuali processi precedenti"
echo "======================================================"

tmux kill-session -t "$SESSION" 2>/dev/null

pkill -f "ign gazebo" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f rviz 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null

sleep 2

echo "Creo sessione tmux..."

tmux new-session -d -s "$SESSION"

GAZEBO_CMD="
source $ROS_SETUP && \
source $WS_SETUP && \
ros2 launch spot_bringup spot.gazebo.launch.py \
world_file:=/home/spot/PycharmProjects/spot_system/worlds/test.sdf
"

echo "Avvio Gazebo..."

tmux send-keys -t "$SESSION" "$GAZEBO_CMD" C-m

echo ""
echo "======================================================"
echo "GAZEBO AVVIATO"
echo ""
echo "1) Attendi che Gazebo si apra"
echo "2) Premi PLAY"
echo "3) Torna qui e premi INVIO"
echo "======================================================"
read

EXPLORATION_CMD="
source $ROS_SETUP && \
source $WS_SETUP && \
cd /home/spot/PycharmProjects/spot_system/launch && \
ros2 launch launch_exploration.launch.py
"

echo "Avvio exploration launch..."

tmux split-window -v -t "$SESSION"

tmux send-keys -t "$SESSION":0.1 "$EXPLORATION_CMD" C-m

echo "Attendo RViz..."

while true
do
    RVIZ_FOUND=$(bash -c "
        source $ROS_SETUP
        source $WS_SETUP
        ros2 node list 2>/dev/null | grep -E '(rviz|rviz2)'
    ")

    if [ -n "$RVIZ_FOUND" ]; then
        break
    fi

    sleep 1
done

echo "RViz rilevato."

echo ""
echo "Log algoritmo:"
echo "$ALGO_LOG"
echo ""

ALGO_CMD="
source $ROS_SETUP && \
source $WS_SETUP && \
cd /home/spot/PycharmProjects/spot_system && \
python3 -m entry_points.run_exploration_ros \
--ros-args \
-p odom_topic:=/spot/odometry \
-p use_sim_time:=true \
2>&1 | tee '$ALGO_LOG'
"

echo "Avvio algoritmo..."

tmux split-window -h -t "$SESSION":0.1

tmux send-keys -t "$SESSION":0.2 "$ALGO_CMD" C-m

tmux select-layout -t "$SESSION" tiled

echo ""
echo "======================================================"
echo "Sistema avviato"
echo ""
echo "Log algoritmo:"
echo "$ALGO_LOG"
echo ""
echo "Comandi utili:"
echo ""
echo "  Ctrl+b z   -> zoom pannello corrente"
echo "  Ctrl+b o   -> pannello successivo"
echo "  Ctrl+b [   -> scroll della cronologia"
echo ""
echo "Per chiudere tutto:"
echo "  tmux kill-session -t $SESSION"
echo "======================================================"
echo ""

tmux attach -t "$SESSION"

cleanup