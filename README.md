# coding_challenge_sereact - Felix Amberg

This project implements a Python-based control system for a stationary 6-axis robot arm. It provides a frontend interface for live control and visualization, supports smooth motion using cubic spline interpolation, and is fully containerized with Docker for easy deployment.

# How to run:

Clone the repository:
```
git clone ....
```
in WSL2 on Windows or Linux:
```
cd coding_challenge_sereact
chmod +x start.sh
./start.sh
```

# How to stop:
docker-compose down

# Example input:
Slide the sliders to desired angle and press 'Move Robot'
Example: [60, 0, 30, 20, 10, 60]

# Notes:
For this example the common 6-axis robot Universal Robot UR5 was used.
To definine the robot and validate it the DH-Parameters were used from the official Universal Robot Website:
[UR6 DH_parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)


# Task 1:
The robot is defined by six Joints each with name and angular limits.
Further additions to that can be additional angular velocity and acceleration limits to ensure those in motion planning.
For this simple simulation, no gravity, no inertia and no friction was considered.

# Task 2:
User input is sent via a control panel on frontend and sent to backend.
The joints limits are both enforced by the Joint, that will not move outside its limits and the interpolation method.

# Task 3:
A smooth trajectory is generated using the cubic spline method.
An additional trajectory generation method using cubic splines to avoid obstacles is not yet functional. The additional method simualtes an initially generated cubic spline trajectry and checks for points in joint space that collide with itself, an obstacle or the floor. If a collision is detected an additional viapoint with a random pertubation in joint-space is created for each collision point. Based on these additional viapoints a new cubic spline is generated and tested again. This process is repeated with increased pertubation range until it finds a feasible path or the limit is reached. 

Other more sophisticated techniques like RRT could possibly be implemented in the future.

# Task 4: 
The backend sends constant updates with the current state of the robot and a 3d visualization is used in the frontend, based on the urdf representation of the Universal Robot ROS package.
[UR ROS Package](https://github.com/ros-industrial/universal_robot)

# Task 5:
Both frontend and backend are dockerized in a separate container and run together using docker compose


