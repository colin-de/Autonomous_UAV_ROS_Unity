# Docker


## Setup
Download the docker file and navigate to its location inside a terminal. Build the image with this command:
```
sudo docker build -t advanced_challenge_1_docker_auto . --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --no-cache 
```
and run it
```
sudo docker run -it -d --network=host --name advanced_challenge_1_docker_auto advanced_challenge_1_docker_auto:latest bash
```

## Starting and interacting
Start the docker container:
```
sudo docker start advanced_challenge_1_docker_auto
```

Open an interactive shell:
```
sudo docker exec -it advanced_challenge_1_docker_auto bash
```

To exit the docker shell type `exit`.

## Stopping and deleting
Stop the container:
```
sudo docker stop advanced_challenge_1_docker_auto
```
Delete the container:
```
sudo docker rm advanced_challenge_1_docker_auto
```

# Simulation instructions

You will need five terminals. Please make sure you have the [prerequisites](https://gitlab.lrz.de/00000000014ACFEA/autonomous-systems-2021-group-auto/-/tree/main/AdvancedChallenge1SimAndMapping#prerequisites) in place, and that you have followed the [installation](https://gitlab.lrz.de/00000000014ACFEA/autonomous-systems-2021-group-auto/-/tree/main/AdvancedChallenge1SimAndMapping#installation) instructions from the main README. This is needed since the simulation and mapping is run outside of Docker, as they depend on Unity and RViz.

### Terminal 1, on host:
```
roscore
```

### Terminal 2, on host:
Open a new terminal under `AdvancedChallenge1SimAndMapping/catkin_ws` and run: 
```
source devel/setup.bash
roslaunch simulation simulation.launch
```

### Terminal 3, on host:
Open a new terminal under `AdvancedChallenge1SimAndMapping/catkin_ws` and run: 
```
source devel/setup.bash
roslaunch mapping mapping.launch
```

### Terminal 4, in docker
Open an interactive shell inside the container and run:
```
source devel/setup.bash
roslaunch controller_pkg controller.launch 
```

### Terminal 5, in docker
Open an interactive shell inside the container and run:
```
source devel/setup.bash
roslaunch navigation navigation.launch 
```
