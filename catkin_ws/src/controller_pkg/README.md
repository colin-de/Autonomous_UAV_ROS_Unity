# Controller
This module will subscribe to the Current State and Trajectory signals and convert the subscribing signals into tracking errors. And the tracking error will be used to calculate the total thrust and total moment. Finally, the rotor speed of the four different propellers can be derived from total thrust and total moment with the help of the [mav_comm package](https://github.com/ethz-asl/mav_comm) from ETH Zurich.

The parameters of controller should be modified regarding the different Unity enviroment. The suited parameters could be changed in [controller_params.yaml](https://gitlab.lrz.de/00000000014ACFEA/autonomous-systems-2021-group-auto/-/blob/main/AdvancedChallenge1SimAndMapping/catkin_ws/src/controller_pkg/config/controller_params.yaml).
