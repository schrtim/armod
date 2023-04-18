#About the project
ARMoD is a project that uses a humanoid robot to communicate the intent of a host robot acting as an “Anthropomorphic Robotic Mock Driver” (ARMoD). The project aims to improve engagement and social interaction with mobile robots in workplace settings.

The project contains a Dockerfile and various scripts to work with it. The code for controlling the NAO robot is found in the “code” subdirectory.

#Getting Started
To get started with ARMoD, first build the Docker image using the build_docker.sh script. Once the image is built, you can run it in any terminal using the run_docker script. If you want to attach the Docker container to a new terminal window or tab, use the attach_docker script.

For the original source of the docker structure used in this repository, refer to: https://github.com/alexsleat/naoposner

#Code
The code for controlling the NAO robot is found in the “code” subdirectory. It includes two scripts: robot_controller and perception_bridge.

The robot_controller script controls the NAO robot using Naoqi and the Python 2.7 SDK. It includes a class called CommandExecuterModule that uses the Naoqi SDK to interact with the robot and control its movements. This class also subscribes to a ROS topic to receive commands.

The perception_bridge script handles the interaction with ROS perception modules in the network and is capable of exchanging ros std_mgs with the Robot Controller script. It includes a class called ArmodCommand that subscribes to a ROS topic to receive perception messages and publishes commands to a different ROS topic. This class also includes a method for sending commands to the robot_controller script.

#Configuration
ARMoD includes a config file that allows you to specify the IP and port to talk to the NAO robot, as well as specifications on certain timing and the Head-Resting Pose of NAO.
