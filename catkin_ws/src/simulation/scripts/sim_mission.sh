#!/usr/bin/env bash

rosrun mavros mavparam set RC_MAP_MODE_SW 7
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
rostopic pub -1 /flight/event fla_msgs/FlightEvent "event_id: 1"