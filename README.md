# The pnp servers repository

This repository contains all the the actions that can be executd by the ROSPlan - PNP execution framework. In the following you can find a list of the actions.

For installation and usage instructions, please visit: http://protolab.aldebaran.com:9000/mummer/documentations/wikis/wp4-demo-system

## PNP actions

* `pepper_check_human_exists`: contains a script that makes sure that the human that was perceived during the generation of the plan is still visible before executing any movement actions.
* `pepper_engage_human` contains the following scripts:
 * `engage_human.py` simply sets the engaged predicate. Used as a final step to achieve the overall goal of engaging a human
 * `terminate_interaction.py` stops tracking of the human porisition and resets a lot of predicates to be able to enagege a new human.
 * `find_interactant.py` sets predicates necessary for planning.
* `pepper_move_base` contains the following scripts:
 * `move_base_emulator.py` emulates the behaviour of move_base, i.e. provides a blocking action server that moves the robot.
 * `go_home.py` returns the robot to the learned home position.
 * `move_to_waypoint.py` moves the robot to a specified waypoint using mongodb to look up the coordinates. These waypoints can be read from the semantic map when looking for a location from where to describe the route to a given shop.
 * `qualitative_move_base.py` moves the robot with respect to the human specified. This takes the id of a detected human and a a qualitative change in position like move from `far` to `close`. The qualitative distances can be defined in the config file provided in this package.
 * `track_human.py` starts or stops tracking of a specific human given their id. Ina dditiona to moving the head, this also turns the robot on the spot to face the human, hence why it is in the move_base package.
* `pepper_route_description` this package contains the following scripts:
 * `describe_route.py` this uses the semantic map to look-up the location of the shop that the robot is supposed to describe the route to, points at that location (using the `semantic_map_tf_publisher`), and says the hard coded string also defined in the semantic map. The robot then turns to face the human again.
 * `finish_description.py` resets a few predicates necessary for planning.
* `pepper_speak` contains a script that starts the dialogue. Dialogue currently not available on protolab, yet.
* `pepper_world_state_kb` this package contains a script that reads several topics and uses the information gathered to update the knowledge base with, e.g. the charging status of the battery, the relative position to the human, if the human is looking at the robot, etc. Please have a look at the README of this package to find out how to use the provided config file to automatically parse any kind of ros topic into KB predicates.

## Other packages

* `pepper_planning_launch` contains launch files and a tmux script to start the whole system. Please, have a look at the README of this package for mor information how to use the tmux script.
* `pepper_planning_control` contains scripts and servers to automatically add goals to the planning system and control the planning loops.
