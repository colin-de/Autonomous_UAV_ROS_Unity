# Simulation
This module is responsible for the simulation of the environment and the drone itself. It contains the unity environment, which is downloaded when running ```catkin build```. To change the unity environment, simply change the Dropbox link in CMakeLists.txt of the simulation package. The following Unity environments are avalable for this project:

- Large environment: ```https://www.dropbox.com/s/urmpmhuu5diidtw/City_Scenary.zip?dl=0```
- Large environment, high FPS: ```https://www.dropbox.com/s/vb0opb5f3e7g7xq/City_Scenary.zip?dl=0```
- Large environment, highest FPS, no vegetation: ```https://www.dropbox.com/s/pakfwkoyvnieipe/City_Scenary.zip?dl=0```
- Small, testing environment:  ```https://www.dropbox.com/s/0q4qdkgupfe19jt/City_Scenary.zip?dl=0```

Unity is used as rendering and physics engine. The graphic setting and physics properties can only be changed using the raw unity files of the project. The raw unity files can be downloaded using the following link:
```https://www.dropbox.com/s/e4onw7ebi0lk351/Unity.zip?dl=0```
To be able to open the raw unity files, Unity needs to be installed.

Unity and ROS communicate over a local TCP-Server which is created in TCPServer.cs. Data from Unity to ROS (sensor data) runs over unity_ros.cpp and data from ROS to Unity (controller data) runs over w_to_unity.cpp The drone is equipped with 4 sensors, for which all the code can be found in the raw unity files as well. The camera setup and the Unity-ROS communication can also be found in the raw unity files. The IP-addresses and the ports used for the Unity-ROS communication can be found in the simulation.yaml file.
