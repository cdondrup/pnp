# The papper planning launch package

This package contains several launch files to start the the components and a tmux script to start the whole system.

## Launch files

`pepper_planning.launch` starts the PNP server and all the pnp actions in this repository. Parameters:

* `semantic_map_name`: The name of the semantic map in mongodb.
* `move_config_file` _Default: `$(find pepper_move_base)/conf/distances.yaml`_: The file defining the qualitative distances for the qualitative_move script.
* `kb_config_file` _Default: `$(find pepper_world_state_kb)/conf/parsing.yaml`_: The config file for the KB world state update script.

`pepper_control.launch` starts all the necessary controll scripts that add goals and take care of the planning. Parameters:

* `semantic_map_name`: The name of the semantic map in mongodb.
* `goal_config_file` _Default: `$(find pepper_goal_server)/conf/goals.yaml`_: The goal server config file.
* `control_config_file` _Default: `$(find pepper_control)/conf/goals.yaml`_: The control server config file.

## Using the tmux script

Pre-requsites

```
sudo apt-get install tmux htop
```

Running the script

```
rosrun pepper_planning_launch pepper_start.sh
```

How to use tmux: [shortcut list](https://gist.github.com/MohamedAlaa/2961058)

this script migfht not work for you directly but requires you to change some of the parameters given to the launch files.
