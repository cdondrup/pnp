#!/bin/bash

SESSION=$USER

check_session=`tmux ls |grep $SESSION`
if [ -n "$check_session" ]; then
	echo -e "\e[1m\e[31m+++ ERROR: tmux session with name $SESSION is already running. +++\e[0m"
	echo "Try:"
	echo
	echo -e "\ttmux a -t $SESSION"
	echo
	echo "to attach to it and either keep on using this session or close all the terminals (Ctrl+C and Ctrl+D) and rerun this script."
	echo
	exit
fi

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux rename-window -t $SESSION:0 'roscore'
tmux new-window -t $SESSION:1 -n 'mongodb'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'wp2'
tmux new-window -t $SESSION:4 -n 'wp3'
tmux new-window -t $SESSION:5 -n 'ros_plan'
tmux new-window -t $SESSION:6 -n 'dialogue'
tmux new-window -t $SESSION:7 -n 'pnp_server'
tmux new-window -t $SESSION:8 -n 'semantic_map_tf'
tmux new-window -t $SESSION:9 -n 'control'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch naoqi_navigation navigation_full.launch"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch wp2_perception pepper_main.launch"

tmux select-window -t $SESSION:4
tmux send-keys "DISPLAY=:0 rosrun wp3_ssp engagement"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 roslaunch rosplan_planning_system planning_system_knowledge.launch domain_path:=$(rospack find rosplan_examples)/share/greet.pddl persistent:=true"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 roslaunch mummer_dialogue_launch mummer_dialogue.launch semantic_map_name:=my_map"

tmux select-window -t $SESSION:7
tmux send-keys "DISPLAY=:0 roslaunch pepper_planning_launch pepper_planning.launch semantic_map_name:=my_map"

tmux select-window -t $SESSION:8
tmux send-keys "DISPLAY=:0 rosrun semantic_map_transform_publisher transform_publisher.py"

tmux select-window -t $SESSION:9
tmux send-keys "DISPLAY=:0 roslaunch pepper_planning_launch pepper_planning_control.launch"

# Set default window
tmux select-window -t $SESSION:0
tmux setw -g mode-mouse off

# Attach to session
tmux -2 attach-session -t $SESSION
