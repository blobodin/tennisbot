# tennisbot

The tennisbot repository is meant to be
run as a ROS module. It contains a number of files
for both and 8DOF and a 7DOF tennisbot. However, only the 7DOF is in working
form. When running tennisbot using the 7DOF setup, please use the
gazebodemo_trajectory.launch file or the gazebo_seven_dynamic_pd.launch file
directly. 

The important files in the scripts folder include:
1. gazebo_trajectory.py which includes the trajectory generator code for the
7DOF tennisbot.
2. ball_kinematics.py which is used when calculating the trajectory of a
tennis ball.
3. ball_launcher.py which includes the relevant code for spawning and launching
a tennis ball.
4. kinematics.py and splines.py which contain helper code used in previous
homework assignments.
