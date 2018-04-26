# -*- coding: utf-8 -*-

import argparse
from math import cos, sin
import math

import numpy as np
import scipy

from pydrake.all import (DiagramBuilder, FloatingBaseType,
                         LinearQuadraticRegulator,
                         RigidBodyTree, RigidBodyPlant,
                         Simulator, SignalLogger, VectorSystem)
from underactuated import (FindResource, ManipulatorDynamics,
                           PlanarRigidBodyVisualizer)

_LINEARIZE_GRAVITY = False

class Controller(VectorSystem):
    """ Defines a feedback controller for the double pendulum.

    The controller applies torques at the joints in order to
    1) cancel out the dynamics of the double pendulum,
    2) make the first joint swing with the dynamics of a single pendulum, and
    3) drive the second joint towards zero.

    The magnitude of gravity for the imposed single pendulum dynamics is taken
    as a constructor argument.  So you can do fun things like pretending that
    gravity is zero, or even inverting gravity!

    """

    def __init__(self, rigid_body_tree, gravity):
        # 4 inputs (double pend state), 2 torque outputs.
        VectorSystem.__init__(self, 4, 2)
        self.tree = rigid_body_tree
        self.g = gravity

        self.m1 = 1.
        self.l1 = 1.
        self.lc1 = self.l1
        self.m2 = 1.
        self.l2 = 1.
        self.lc2 = self.l2

        self.K, self.S = self._LQR()
        q_f = np.array([math.pi, 0])
        qd_f = np.array([0, 0])
        # print(self.LQR)

    def evaluate_f(self, x, u):
        # Use the manipulator equation to get qdd.
        q = x[0:2]
        qd = x[2:4]
        u = np.array([u, 0])
        (M, C, tauG, B) = ManipulatorDynamics(self.tree, q, qd)

        # Awkward slice required on tauG to get shapes to agree --
        # numpy likes to collapse the other dot products in this expression
        # to vectors.
        qdd = np.dot(np.linalg.inv(M), (tauG + np.dot(B, u) - np.dot(C, qd)))
        return np.hstack([qd, qdd])

    def evaluate_linearized_f(self, x, u):
        x = np.array(x)
        x[:2] -= np.array([math.pi, 0])
        x[:2] = np.unwrap(x[:2])
        A, B = self._GetLinearizedDynamics()
        return np.dot(A, x) + B.flatten()*u

    def _GetLinearizedDynamics(self):
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        q_f = np.array([math.pi, 0])
        qd_f = np.array([0, 0])

        (M, C_f, tauG_f, B_f) = ManipulatorDynamics(self.tree, q_f, qd_f)

        A = np.zeros((4, 4))
        B = np.zeros((4, 1))
        dtau_g = np.zeros((2,2))
        dtau_g[0,0] = m1*g*lc1 + m2*g*l1
        dtau_g += m2*g*lc2
        if not _LINEARIZE_GRAVITY:
            dtau_g[0,:] = 0
        A[:2,2:] = np.eye(2)
        A[2:,:2] = dtau_g
        B[2:,:1] = B_f[:,:1]
        M_x = np.eye(4)
        M_x[2:,2:] = M
        # print(A, B, M_x)
        A = np.linalg.lstsq(M_x, A)[0]
        B = np.linalg.lstsq(M_x, B)[0]
        return (A, B)

    def _LQR(self):
        A, B = self._GetLinearizedDynamics()
        # print(A, B)
        rot = np.array([
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
        ])
        # Q in (theta_1, theta_1+theta_2, theta_1d, theta_1d+theta_2d) basis
        Q = np.array([
            [30, 0, 0, 0],
            [0, 500, 0, 0],
            [0, 0, 30, 0],
            [0, 0, 0, 200],
        ])
        Q = np.dot(rot.T, np.dot(Q, rot))
        R = np.array([[1e-1]])
        # K, S = LinearQuadraticRegulator(A, B, Q, R)
        S = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.lstsq(R, np.dot(B.T, S))[0]
        # print(np.dot(S, A) + np.dot(A.T, S) - np.dot(S, np.dot(B, np.dot(R**(-1), np.dot(B.T, S)))))
        # K = np.array([[-149.84436802, -136.09344666,  -63.96348586,  -40.20875713]])
        return (K, S)

    def lqr_controller(self, x):
        q = x[:2]
        v = x[-2:]
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        x = np.array(x)
        x[0] -= np.pi
        x[:2] = np.unwrap(x[:2])
        K, S = self.K, self.S
        u_err = np.dot(-K, x).item()
        if _LINEARIZE_GRAVITY:
            u = u_err
        else:
            # tau_sys = np.linalg.lstsq(M, -Cv + tauG)[0]
            tau_sys = -Cv + tauG
            u = u_err - tau_sys[0]
        return u

    def _GetTorque(self, state):
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        # Extract manipulator dynamics.
        q = state[:2]
        v = state[-2:]

        if False:
            (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

            theta_1 = M[0,0] - M[0,1] - M[1,0] + M[1,1]
            theta_2 = M[1,1]
            # theta_3 = (M[0,1] - theta_2) / cos(q[1])
            # theta_5 = tauG[1] / (g * sin(q[0] + q[1]))
            # theta_4 = (tauG[0] - tauG[1]) / (g * sin(q[0]))
            theta_3 = m2 * l1 * (l2/2)
            theta_4 = m1 * (l1/2) + m2 * l1
            theta_5 = m2 * (l2/2)

            E = 1./2 * np.dot(v.T, np.dot(M, v)) + theta_4 * g * -cos(q[0]) + theta_5 * g * -cos(q[0]+q[1])
            E_top = theta_4 * g + theta_5 * g
            E_curl = E - E_top

            k_p = 5.5
            k_d = 0.5
            k_e = 1

            if abs(E_curl) < min(2 * theta_4 * g, 2 * theta_5 * g, k_d / (k_e * theta_1)):
                # stability controller

                f = theta_2 * theta_3 * (v[0]+v[1])**2 * sin(q[1]) \
                    + theta_3**2 * v[0]**2 * cos(q[1]) * sin(q[1]) \
                    - theta_2 * theta_4 * g * sin(q[0]) \
                    + theta_3 * theta_5 * g * cos(q[1]) * sin(q[0]+q[1])

                q0_d = math.pi
                q_curl = q[0] - q0_d

                tau = (-k_d * f - (theta_1 * theta_2 - theta_3**2 * cos(q[1])**2) * (v[0] + k_p * q_curl)) \
                    / ((theta_1 * theta_2 - theta_3**2 * cos(q[1])**2) * k_e * E_curl + k_d * theta_2)

                p = np.dot(M, v)
                du = -tauG * v

            else:
                # swing up  controller
                tau = 0

        tau = self.lqr_controller(state)


        tau_limit = 50
        tau = np.clip(tau, -tau_limit, tau_limit)
        # print(tau)
        return tau
        # return np.sign(np.dot([1,-1], np.dot(M, v)))

    def _DoCalcVectorOutput(self, context, double_pend_state, unused, torque):
        # Extract manipulator dynamics.
        # q = state[:2]
        # v = state[-2:]

        # Control gains for stabilizing the second joint.
        # kp = 1
        # kd = .1

        # Desired pendulum parameters.
        # length = 2.
        # b = .1

        # Cancel double pend dynamics and inject single pend dynamics.
        # torque[:] = Cv - tauG + \
            # M.dot([-self.g / length * math.sin(q[0]) - b * v[0],
                   # -kp * q[1] + kd * v[1]])

        torque[0] = self._GetTorque(double_pend_state)
        torque[1] = 0

def make_controller(gravity=9.8):
    tree = RigidBodyTree(FindResource("double_pendulum/double_pendulum.urdf"),
                         FloatingBaseType.kFixed)
    return Controller(tree, gravity)


def SimulatePendulum(x0, duration, gravity=9.8):
    # Load the double pendulum from Universal Robot Description Format
    tree = RigidBodyTree(FindResource("double_pendulum/double_pendulum.urdf"),
                         FloatingBaseType.kFixed)

    # Set up a block diagram with the robot (dynamics), the controller, and a
    # visualization block.
    builder = DiagramBuilder()
    robot = builder.AddSystem(RigidBodyPlant(tree))

    controller = builder.AddSystem(Controller(tree, gravity))
    builder.Connect(robot.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), robot.get_input_port(0))

    # Create a logger to log at 30hz
    state_log = builder.AddSystem(SignalLogger(robot.get_num_states()))
    state_log._DeclarePeriodicPublish(0.0333, 0.0) # 30hz logging
    builder.Connect(robot.get_output_port(0), state_log.get_input_port(0))

    input_log = builder.AddSystem(SignalLogger(tree.get_num_actuators()))
    input_log._DeclarePeriodicPublish(0.0333, 0.0) # 30hz logging
    builder.Connect(controller.get_output_port(0), input_log.get_input_port(0))

    # visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                             # xlim=[-2.8, 2.8],
                                                             # ylim=[-2.8, 2.8]))
    # builder.Connect(robot.get_output_port(0), visualizer.get_input_port(0))

    # Set up a simulator to run this diagram
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Set the initial conditions
    context = simulator.get_mutable_context()
    state = context.get_mutable_continuous_state_vector()
    state.SetFromVector(x0)  # (θ₁, θ₂, θ̇₁, θ̇₂)

    # Simulate
    simulator.StepTo(duration)

    return tree, controller, state_log, input_log
