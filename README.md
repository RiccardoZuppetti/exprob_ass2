# Experimental Robotics Laboratory - Assignment 2

The goal of this assignment is to develop a software architecture able to simulate an autonomous Cluedo game. In particular the robot has to explore an environment looking for hints that are generated in established locations. These locations can be reached via the robotic arm provided by the model of the robot used. In order to control the arm and reach the hints, a dedicated `moveit_assignment` package has been developed. As soon as the robot is ready to make an hypothesis, it moves towards the center of the environment, asking to the oracle if the considered hypothesis is correct or not. The behaviour of the robot is based on a planning system, since the robot's actions are selected through a `PDDL` planning system.

## Compatibility

The project is developed using the provided docker container. In the event that it is not possible to test the project on this image, it is also possible to use ROS Noetic on Ubuntu 20.04.2, after installing the ARMOR and Moveit components on this operating system. In any case, the project is compatible with ROS Noetic, and consequently may not work using a different ROS distribution.

# Description of the package

## Robot behaviour and ROSPlan architecture

Referring to the main purpose of the project, the robot should perform the following actions:

1. move around the aforementioned environment
2. move its arm in order to reach the established locations 
3. perceive an hint as soon as it has reached the established location
4. check the consistency of an hypothesis
5. check if an hypothesis is correct

Each one of the above actions refer to a specific robot's state. The order in which these actions should be performed corresponds to the plan generated by the ROSPlan system. Referring to this system, the dispatcher is addressed to perform transitions between different states. To these states it is possible to add:
- an initial state of the robot
- an announce state, which is strictly related to the check correct state

All these states are connected through a reasoning module, that implements ROSPlan. In the `common` folder it is possible to find the defined `PDDL` [domain](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/common/cluedo_domain_nohint.pddl) and the relative [problem](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/common/cluedo_problem_nohint.pddl). In the domain file, the following types have been defined:

- robot
- location
- hypothesis

On top, here each state of the robot corresponds to a durative action that has to be accomplished:

- move
- move_gripper
- perceive_hint
- check_consistency
- check_correct
- new_turn (action used to make the robot visit again all the waypoints after that a turn of navigation has been completed)

These durative actions are used to manage the following predicates:

- visited (and non_visited), used to manage the navigation
- not_initial_location, used to differentiate the initial location and the waypoints
- at, used to state the actual location
- move_performed, used to move the robot's arm when it has reached a location
- gripper_positioned (and not_gripper_positioned), used to state the gripper has (not) moved correctly and the hint can (cannot) be perceived 
- perceived, used to state if an hypothesis has been perceived
- consistent, used to state if a consistent hypothesis has been found
- end_game, used to state that a correct hypothesis has been found and the game is ended

The related problem file, instead, initialize the robot at the center of the environment, and all the waypoints are set as non_visited. The goal of the problem is to have the predicate end_game as true, i.e., that a correct hypothesis has been found.

## ROS Nodes

The nodes are identifiable through the files inside the `src` and the `scripts` folder, and so we have:

- [ArmorInterface.py](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/scripts/ArmorInterface.py), a service node that interacts with ARMOR in different ways: load the ontology; add a new hint to the ontology; check that an hypothesis is consistent; check that an hypothesis is correct 
- [AnnounceHypotesis.py](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/scripts/AnnounceHypotesis.py), a service node used for the announcement of the robot: once the robot has found a consistent hypothesis, the robot moves toward the center of the environment and announce it to the oracle
- [Reasoning.py](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/scripts/Reasoning.py), the node that manages the entire simulation: first of all, it asks to the [ArmorInterface.py](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/scripts/ArmorInterface.py) to load the ontology; then the first phase of the game is started; since the game is not ended it interfaces with the ROSPlan system, recalling the problem loader, the plan generator, the parse planning service and the dispatcher to dispatch the actions
- [simulation.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/simulation.cpp), a node to position randomly the locations where hints are perceived; on top it generates hints randomly and publish them on the `oracle_hint` topic. These hints can be correctly perceived or malformed by the [ArmorInterface.py](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/scripts/ArmorInterface.py). Moreover the oracle responds if the hypothesis to check is correct or not.
- [initial_phase.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/initial_phase.cpp), a service node used to simulate the initial phase of the game

The actions called by the dispatcher are implemented as classes, defined in the `include/my_erl2` folder. In the following it is possible to find the implemented actions:

- [move.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/move.cpp) implements the action devoted to the motion of the robot, that relies on this [script](https://github.com/CarmineD8/rt2_packages/blob/main/motion_plan/scripts/go_to_point_action.py)
- [grippermotion.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/grippermotion.cpp) implements the action to move the robotic arm: in order to reach a certain location in the environment, the plan and the motion of the robotic arm is directly managed by moveit through the packege `moveit_assignment`
- [perceivehints.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/perceivehints.cpp), [checkconsistency.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/checkconsistency.cpp) and [checkcorrect.cpp](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/src/checkcorrect.cpp) are three action nodes that simply recall the [ArmorInterface.py](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/scripts/ArmorInterface.py) script respectively to perceive a new hint, check if there is a new consistent hypothesis, check if the current consistent hypothesis is correct.

Instead the services are:

- [ArmorInterface.srv](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/srv/ArmorInterface.srv), which has as request the mode that the client wants in order to interact with ARMOR (0 to load ontology, 1 to check correct, 2 to check consistency, 3 to perceive an hint) and the ID of the hypothesis to be checked. The response is composed by the same mode and ID in addition to a success field, which is true if the action has been accomplished correctly.
- [Announcement.srv](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/srv/Announcement.srv), which has as request an hypothesis (composed of [who, what, where]), while the response is a boolean to state that the action has been completed correctly.

Finally the message is:

- [ErlOracle.msg](https://github.com/RiccardoZuppetti/exprob_ass2/blob/main/my_erl2/msg/ErlOracle.msg), which is used to store a perceived hint

## States diagram



## UML diagram

![UML_2](https://user-images.githubusercontent.com/89387809/199231588-3ade1cd1-e29a-4c11-b00a-0faf38fe634a.jpg)

## Rqt-graph

![rqt_graph](https://user-images.githubusercontent.com/89387809/199276290-dde7ac8b-ea2f-4122-b9fe-79df435cab74.png)

# How to Run

## Requirements

In order to compile this project, the following ROS packages are needed:

- [armor](https://github.com/EmaroLab/armor)
- [motion_plan](https://github.com/CarmineD8/rt2_packages/tree/main/motion_plan)
- [ROSPlan](https://github.com/KCL-Planning/ROSPlan)

Then, in order to install Moveit, follow this [guide](https://github.com/RiccardoZuppetti/move_it_installation/blob/main/instr.txt).

## How to compile the code

Move to your `catkin_ws/src/armor/armor` folder and digit (only the first time):

```
./gradlew deployApp
```

Then, move to your `catkin_ws/src` folder and clone this repository:

```
git clone https://github.com/RiccardoZuppetti/exprob_ass2.git experimental_assignment2
```

and build the workspace with the following command:

```
catkin_make
```

In case you get errors about missing packages, proceed with the installation of those packages by typing:

```
apt-get install ros-noetic-<missing_package>
```

## Description of the execution

To launch the project it is needed to open four shells.

In the first shell, digit:

```
roscore &
```

and then

```
rosrun armor execute it.emarolab.armor.ARMORMainService
```

In the second one digit:

```
roslaunch moveit_assignment gazebo_launcher.launch 2>/dev/null
```

In the third terminal digit the following command:

```
roslaunch my_erl2 pddl_launcher.launch 2>/dev/null
```

Finally, in the last shell digit:

```
roslaunch my_erl2 game_launcher.launch 2>/dev/null
```

In the following it is possible to observe several screenshots that refer to the execution of the project.

<img width="1399" alt="shell_1" src="https://user-images.githubusercontent.com/89387809/199282915-1e671015-e592-46e6-8045-49db664713a6.png">

<img width="1402" alt="shell_2" src="https://user-images.githubusercontent.com/89387809/199282922-17cd091d-009d-428c-be0f-4206fd72d205.png">

<img width="1403" alt="shell_3" src="https://user-images.githubusercontent.com/89387809/199282931-c8d6a75b-c1c7-486f-8843-438a218bf6ba.png">

# Working hypothesis and environment

## System's features

The system is characterized by a modular architecture, since each component has a specific role in the architecture.

As can be seen in the nodes, services and actions are the communication protocols used. This means that most of the declared modules are sincronized with the others. 

The ontology is the core of the whole game. It allows to reason about hypotesis. The hypothesis are make of hints. Hints can belong to three different classes: PERSON, PLACE and WEAPON. The perceived hints as ErlOracle message are the A-box of these concepts. The hypotesis belong to the COMPLETE class if they has at least one PERSON, one WEAPON and one PLACE, while they belong also to the INCONSISTENT class if they have more than one PERSON or PLACE or WEAPON. Consequently the consistent hypotesis are the ones that has only one entity for each hint class and they are the ones which belong to the COMPLETE class and not to the INCOSISTENT one. The INCORRECT class, instead, has as instances all the hypothesis already checked as not-correct.

The ROSPlan system manages the whole simulation. This firstly works as PDDL planning system which generate a plan in order to reach the goal in the optimal way. ROSPlan is basically the interface between the PDDL file and ROS. This allows to syncronize the PDDL problem, the execution of the PDDL's durative actions and the corresponding actions in ROS. ROSPlan guarantee optimal performances with respect to a simple finite state machine.

The robotic arm is controlled by moveit, that directly compute the inverse kinematic, find the optimal plan with RRT to reach a certain goal and execute it.

## System's limitations

Regarding the navigation module is not possible to control the yaw of the robot, since in the `go_to_point_action.py` script the final yaw is always zero with respect to the segment that goes from the starting location to the final one. So if the robot goes directly in two adjacent verteces of the environment the final yaw not always allows to perceive the hint. This issue has been overcomed making sure that the robot always returns in the (0, 0) position.

After a certain number of iterations the arm does not reach the goal posistion and remains stopped even if the related action concludes correctly. In order to not make impact this behavior, a control is done on the perceived hints. If the same hint has been perceived twice that means that the arm does not move and consequently the related action fails.

## System's technical improvements

Referring to the above issue of the yaw during the navigation, the desired yaw in the `go_to_point_action.py` script must be modified, or another navigation planning system can be used, such as move base.

An improvement can be done on the robot model, since the robot has been done primitives shapes with only the strigtly needed requirements.

Since the initial phase, which is responsible of load the ontology and explore the environment, is not managed through ROSPlan, a possible solution is to add a specific durative-action on the domain file. A predicate which is grounded only once the action is concluded and this predicate should be added as precondition of all other actions. Logically the ROS Plan will be adapted consequently.

# Author and contacts

Author: Riccardo Zuppetti

Contacts: riccardo.zuppetti@icloud.com
