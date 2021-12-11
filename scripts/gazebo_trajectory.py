#!/usr/bin/env python3
#
#   gazebodemo_trajectory.py
#
#   Create a motion, to send to Gazebo for the sevenDOF.
#
#   Publish:   /sevendof/j1_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j2_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j3_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j4_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j5_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j6_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j7_pd_control/command    std_msgs/Float64
#
import rospy
import numpy as np
import random

from sensor_msgs.msg     import JointState
from std_msgs.msg        import Float64
from urdf_parser_py.urdf import Robot

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz
# We could also import the whole thing ("import kinematics"),
# but then we'd have to write "kinematics.p_from_T()" ...

# Import the Spline stuff:
from splines import  CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5, GotoConst

# Import tennisball stuff:
from ball_launcher import *
from ball_kinematics import Ball_Kinematics

# Import ball swing helper class
from swing_helper import Swing_Helper

from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # The Gazebo controllers treat each joint seperately.  We thus
        # need a seperate publisher for each joint under the topic
        # "/BOTNAME/CONTROLLER/command"...
        self.N    = 7
        self.pubs = []
        for i in range(self.N):
            topic = "/seven_arm/j" + str(i+1) + "_pd_control/command"
            self.pubs.append(rospy.Publisher(topic, Float64, queue_size=10))

        # # We used to add a short delay to allow the connection to form
        # # before we start sending anything.  However, if we start
        # # Gazebo "paused", this already provides time for the system
        # # to set up, before the clock starts.
        # rospy.sleep(0.25)

        # Find the simulation's starting position.  This will block,
        # but that's appropriate if we don't want to start until we
        # have this information.  Of course, the simulation starts at
        # zero, so we can simply use that information too.
        msg = rospy.wait_for_message('/seven_arm/joint_states', JointState);
        theta0 = np.array(msg.position).reshape((-1,1))
        rospy.loginfo("Gazebo's starting position: %s", str(theta0.T))

        # IF we wanted to do kinematics:
        # # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()
        # # Instantiate the Kinematics
        self.kin = Kinematics(robot, 'world', 'tip')

        # Set the ready state
        self.ready_state = np.array([-np.pi/4, -np.pi/4, -np.pi/4, np.pi/2, np.pi/4, np.pi/4, np.pi/4]).reshape((7,1))
        (T,J) = self.kin.fkin(self.ready_state)
        self.ready_p = p_from_T(T)
        self.ready_R = R_from_T(T)

        # Set the fore hand state
        self.fore_hand = np.array([-np.pi, 0.0, -np.pi/2, np.pi/4, np.pi/4, np.pi/4, np.pi/4]).reshape((7,1))
        (T,J) = self.kin.fkin(self.fore_hand)
        self.fore_p = p_from_T(T)
        self.fore_R = R_from_T(T)

        # Set the back hand state
        self.back_hand = np.array([2.0, 0.0, -1.4, 1.28, 1.0, -0.25, 0.25]).reshape((7,1))
        (T,J) = self.kin.fkin(self.back_hand)
        self.back_p = p_from_T(T)
        self.back_R = R_from_T(T)

        # Set launched tennis ball conditions

        # Normal forehand normal speed
        ball_p_i = [0.80, 5, .9]
        ball_v_i = [0, -5, 4.5]

        # Normal backhand normal speed
        ball_p_i = [-0.80, 5, .9]
        ball_v_i = [0, -5, 4.5]

        # Faster forehand normal position
        ball_p_i = [0.8, 5, .3]
        ball_v_i = [0, -8, 4.5]

        # Stretched/high forehand normal speed
        ball_p_i = [1.05, 5, 1.5]
        ball_v_i = [0, -5, 4.5]

        # Too far forehand
        ball_p_i = [1.5, 5, 2.5]
        ball_v_i = [0, -5, 4.5]

        # Low forehand normal speed
        ball_p_i = [0.80, 5, 0.4]
        ball_v_i = [0, -5, 4.5]

        # Launch ball randomly
        # ball_p_i = [random.randrange(0,2), random.randrange(0,5), random.randrange(0,5)]
        # ball_v_i = [random.randrange(-2,0), random.randrange(-5,0),random.randrange(0,5)]

        # Setup ball kinematics using initial conditions
        self.ball_kin = Ball_Kinematics(ball_p_i, ball_v_i)
        self.hit_time = self.ball_kin.compute_time_intersect_x()
        self.hit_point = self.ball_kin.compute_pos(self.hit_time)

        # Adjust hit point to lie inside truncated sphere.
        # First raise it to bottom of truncated sphere if it is too low.
        self.hit_point[2, 0] = max(self.hit_point[2, 0], 0.26)

        # Now rescale such that it lies on the surface of the sphere
        # if it is too far out
        center = np.array([0, 0, 0.6]).reshape((3, 1))
        radius = self.hit_point - center
        mag = np.linalg.norm(radius)
        if mag > 1.06:
            radius = (1.06 / mag) * radius
            self.hit_point = center + radius

        # Spawn the ball
        delete_ball()
        spawn_ball(ball_p_i)
        launch_ball(ball_p_i, ball_v_i)

        print(self.hit_time)

        # Create the trajectory segments.  When the simulation first
        # turns on, the robot sags slightly due to it own weight.  So
        # we start with a 2s hold to allow any ringing to die out.
        if self.hit_point[0,0] >= 0:
            # Hold in ready state, move to fore hand position, then swing
            self.segments = [
                Hold(self.ready_state, self.hit_time * (4/9)),
                Goto(self.ready_state, self.fore_hand, self.hit_time * (4/9)),
                GotoConst(0.0, 1.0, self.hit_time * (2/9), space = "Path")
            ]
            # Set p and R to forehand p and R for use later
            self.p = self.fore_p
            self.R = self.fore_R

            # Set rotation of paddle at hit point to be parallel to
            # the xz plane and tilted such that the paddle points
            # in the same direction at the hit point vector.
            self.rot = Rz(-np.pi/2) @ Rx(np.arctan2(radius[2, 0], radius[0, 0]))
        else:
            # Hold in ready state, move to back hand position, then swing
            self.segments = [
                Hold(self.ready_state, self.hit_time * (4/9)),
                Goto(self.ready_state, self.back_hand, self.hit_time * (4/9)),
                GotoConst(0.0, 1.0, self.hit_time * (2/9), space = "Path")
            ]
            # Set p and R to backhand p and R for use later
            self.p = self.back_p
            self.R = self.back_R

            # Set rotation of paddle at hit point to be parallel to
            # the xz plane and tilted such that the paddle points
            # in the same direction at the hit point vector.
            self.rot = Rz(np.pi/2) @ Rx(np.pi - np.arctan2(radius[2, 0], radius[0, 0]))

        # Setup swing helper for swing trajectories
        self.swing_helper = Swing_Helper(self.p, self.ready_p, self.hit_point)

        # Set lasttheta to the initial theta configuration
        self.lasttheta = theta0

        # Setup a slerp object for the interpolation of our desired
        # rotation matrices
        R_set = Rotation.from_matrix(np.array([self.R, self.rot, self.ready_R]))
        self.slerp = Slerp([0, 0.5, 1], R_set)

        # Initialize/save the parameter.
        self.lam = .01

        # Also reset the trajectory, starting at the beginning.
        self.reset()

    # Reset.  If the simulation resets, also restart the trajectory.
    def reset(self):
        # Just reset the segment index counter and start time to zero.
        self.t0    = 0.0
        self.index = 0

    # Path
    def pd(self, s):
        if s <= 0.5:
            y_max = self.hit_point[1, 0]
            y_min = self.p[1, 0]
            traj_type = "fore/back->hit"
            yd = (y_max - y_min) * (2 * s) + y_min
        elif s > 0.5 and s <= 1:
            y_max = self.ready_p[1, 0]
            y_min = self.hit_point[1, 0]
            traj_type = "hit->ready"
            yd = (y_max - y_min) * (2 * (s - 0.5)) + y_min

        pxyd = self.swing_helper.compute_2d_parabola(yd, traj_type)
        pd = self.swing_helper.project_onto_plane(pxyd)

        return pd

    def vd(self, s, sdot):
        if s <= 0.5:
            y_max = self.hit_point[1, 0]
            y_min = self.p[1, 0]
            a = self.swing_helper.a1
            xd_dot = 8 * a * (y_max - y_min)**2 * s * sdot + 4 * a * (y_max - y_min) * y_min * sdot
        elif s > 0.5 and s <= 1:
            y_max = self.ready_p[1, 0]
            y_min = self.hit_point[1, 0]
            a = self.swing_helper.a2
            xd_dot = 8 * a * (y_max - y_min)**2 * (s - 0.5) * sdot + 4 * a * (y_max - y_min) * y_min * sdot

        yd_dot = 2 * (y_max - y_min) * (sdot)
        norm_vec = self.swing_helper.norm_vec
        ref_point = self.swing_helper.ref_point

        zd_dot = -(norm_vec[0, 0] / norm_vec[2, 0]) * xd_dot - (norm_vec[1, 0] / norm_vec[2, 0]) * yd_dot
        return np.array([xd_dot, yd_dot, zd_dot]).reshape((3, 1))

    def Rd(self, s):
        # Return rotation matrix that correponds to step s in the interpolation
        return self.slerp([s]).as_matrix()[0]

    def wd(self, s, sdot):
        adj = s + 0.05
        if s + 0.05 > 1:
            adj = 1

        q2 = self.slerp([adj]).as_quat()[0]
        q1 = self.slerp([s]).as_quat()[0]

        diff = q2 - q1
        conj = np.array([q2[0], -q2[1], -q2[2], -q2[3]])
        vel = np.multiply(((diff * 2) / 0.05), conj)

        return vel[1:].reshape((3, 1))

    # Error functions
    def ep(self, pd, pa):
        return (pd - pa)

    def eR(self, Rd, Ra):
        return 0.5*(np.cross(Ra[:,0:1], Rd[:,0:1], axis=0) +
                    np.cross(Ra[:,1:2], Rd[:,1:2], axis=0) +
                    np.cross(Ra[:,2:3], Rd[:,2:3], axis=0))

    def qdot_sec(self, lam, thetas):
        a1 = 1
        a2 = 1
        a3 = 4

        # Shoulder (-pi <= theta3 <= 0)
        qdot_sec3 = -lam * 2 * a1 * (thetas[2, 0] + np.pi/2)

        # Elbow (0 <= theta4 <= pi)
        qdot_sec4 = -lam * 2 * a2 * (thetas[3, 0] - np.pi/2)

        # Wrist (-pi/4 <= theta7 <= pi/4)
        qdot_sec7 = -lam * 2 * a3 * (thetas[6, 0])

        bound_qdotsec = np.array([0, 0, qdot_sec3, qdot_sec4, 0, 0, qdot_sec7]).reshape((7, 1))

        # Minimize torque
        center    = [0.0, 0.0, 0.0, np.pi/2, 0.0, 0.0, 0.0]
        center    = np.array(center).reshape((-1,1))
        tor_qdotsec = - lam * (thetas - center)

        return bound_qdotsec + tor_qdotsec


    # Update is called every 10ms of simulation time!
    def update(self, t, dt):
        # If the current trajectory segment is done, shift to the next.
        dur = self.segments[self.index].duration()
        if (t - self.t0 >= dur):
            print("Segment change time:", t)
            self.t0    = (self.t0    + dur)
            # self.index = (self.index + 1)                       # not cyclic!
            self.index = (self.index + 1) % len(self.segments)  # cyclic!

            # If this is the first time we finish our segments list,
            # add a reset segment to go back to the ready state.
            if self.index == 0 and len(self.segments) == 3:
                self.segments.append(Goto(self.lasttheta, self.ready_state, 0.5))
                self.index = 3


        # Check whether we are done with all trajectory segments.
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return
        # Decide what to do based on the space.
        if (self.segments[self.index].space() == 'Joint'):
            # Grab the spline output as joint values.
            (theta, thetadot) = self.segments[self.index].evaluate(t - self.t0)

        elif (self.segments[self.index].space() == 'Path'):
            # Determine the desired tip position/rotation/velocities (task
            # information) for the current time.  Start grabbing the
            # current path variable (from the spline segment).  Then use
            # the above functions to convert to p/R/v/w:
            (s, sdot) = self.segments[self.index].evaluate(t - self.t0)

            pd = self.pd(s)
            Rd = self.Rd(s)
            vd = self.vd(s,sdot)
            wd = self.wd(s,sdot)

            # Then start at the last cycle's joint values.
            theta = self.lasttheta

            # Compute the forward kinematics (using last cycle's theta),
            # extracting the position and orientation.
            (T,J) = self.kin.fkin(theta)
            p     = p_from_T(T)
            R     = R_from_T(T)

            print(J)

            # Stack the linear and rotation reference velocities (summing
            # the desired velocities and scaled errors)
            # Note that we use a larger lambda for the rotation error
            xrdot = np.vstack((vd + self.lam * self.ep(pd, p),
                               wd + 50 * self.eR(Rd, R)))

            # Weighted inverse
            g = 0.05
            inv = np.linalg.inv(J.T @ J + g**2 * np.eye(J.shape[1])) @ J.T

            # Secondary tasks
            qsecdot = self.qdot_sec(.01, theta)
            thetadot = inv @ xrdot + (1 - inv @ J) @ qsecdot

            # Take an IK step, using Euler integration to advance the joints.
            # thetadot = inv @ xrdot
            theta    = theta + dt * thetadot



        # Save the joint values (to be used next cycle).
        self.lasttheta = theta

        # # Collect and send the JointState message (with the current time).
        # cmdmsg = JointState()
        # cmdmsg.name         = ['theta1', 'theta2', 'theta3',
        #                        'theta4', 'theta5', 'theta6']
        # cmdmsg.position     = theta
        # cmdmsg.velocity     = thetadot
        # cmdmsg.header.stamp = rospy.Time.now()
        # self.pub.publish(cmdmsg)

        # Send the individal angle commands.
        for i in range(self.N):
            self.pubs[i].publish(Float64(theta[i]))


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('trajectory')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).  This
    # relies on rospy.Time, which is set by the simulation.  Therefore
    # slower-than-realtime simulations propagate correctly.
    starttime = rospy.Time.now()
    lasttime  = starttime
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t  = (servotime - starttime).to_sec()
        dt = (servotime - lasttime).to_sec()
        lasttime = servotime

        # Update the controller.
        generator.update(t, dt)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.  Note, if you reset the
        # simulation, the time jumps back to zero and triggers an
        # exception.  If desired, we can simple reset the time here to
        # and start all over again.
        try:
            servo.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # Reset the time counters, as well as the trajectory
            # generator object.
            rospy.loginfo("Resetting...")
            generator.reset()
            starttime = rospy.Time.now()
            lasttime  = starttime
