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

_LINEARIZE_GRAVITY = True

def mod_angle(x):
    return np.mod(x + np.pi, 2*np.pi) - np.pi

class Controller(VectorSystem):
    """
    Defines a feedback controller for the double pendulum.
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
        self.K_half, self.S_half = self._LQR_half()
        q_f = np.array([math.pi, 0])
        qd_f = np.array([0, 0])
        # print(self.LQR)

        self.lqr_cond = False
        self.extend_cond = False
        self.half_cond = False

    def evaluate_f(self, x, u):
        # Use the manipulator equation to get qdd.
        q = x[0:2]
        qd = x[2:4]
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, qd)

        qdd = np.linalg.lstsq(M, tauG + B[:,0] * u - Cv)[0]
        return np.hstack([qd, qdd])

    def evaluate_linearized_f(self, x, u):
        x = np.array(x)
        x[:2] -= np.array([math.pi, 0])
        x[:2] = mod_angle(x[:2])
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
            [10, 0, 0, 0],
            [0, 200, 0, 0],
            [0, 0, 10, 0],
            [0, 0, 0, 80],
        ])
        Q = np.dot(rot.T, np.dot(Q, rot))
        R = np.array([[1e-1]])
        # K, S = LinearQuadraticRegulator(A, B, Q, R)
        S = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.lstsq(R, np.dot(B.T, S))[0]
        # print(np.dot(S, A) + np.dot(A.T, S) - np.dot(S, np.dot(B, np.dot(R**(-1), np.dot(B.T, S)))))
        # K = np.array([[-149.84436802, -136.09344666,  -63.96348586,  -40.20875713]])
        return (K, S)

    def _LQR_half(self):
        A, B = self._GetLinearizedDynamics()
        A = A[::2,::2]
        B = B[::2]
        Q = np.array([
            [10, 0],
            [0, 2],
        ])
        R = np.array([[1e-1]])
        S = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.lstsq(R, np.dot(B.T, S))[0]
        return (K, S)

    def lqr_controller(self, x):
        q = x[:2]
        v = x[-2:]
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        x = np.array(x)
        x[0] -= np.pi
        x[:2] = mod_angle(x[:2])
        K, S = self.K, self.S
        if abs(x[0]) < np.pi/2 and abs(x[0] + x[1]) < np.pi/2:
            u_err = np.dot(-K, x).item()
        else:
            u_err = 0
        if _LINEARIZE_GRAVITY:
            a_sys = np.linalg.lstsq(M, -Cv)[0]
            u_sys = -a_sys[0] / np.linalg.lstsq(M, B[:,0])[0][0]
            u = u_err + u_sys
        else:
            a_sys = np.linalg.lstsq(M, -Cv + tauG)[0]
            u_sys = -a_sys[0] / np.linalg.lstsq(M, B[:,0])[0][0]
            # print(x, u_err, u_sys, tauG)
            u = u_err + u_sys
        # print(q, v, u)
        return u

    def extend_controller(self, x):
        q = x[:2]
        v = x[-2:]
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        x = np.array(x)
        x[0] -= np.pi
        x[:2] = mod_angle(x[:2])
        q = x[:2]
        v = x[-2:]

        tau_limit = 50
        if q[0] * (q[0]+q[1]) > 0:
            tau = np.sign(q[0]) * tau_limit
        elif q[0] * v[0] > 0:
            tau = -np.sign(v[0]) * tau_limit
        # elif q[0] > 0.01 and abs(v[0] / q[0]) < 0.05 and abs(q[0] + q[1]) / min(np.pi/4, abs(q[0])) < 1./8:
            # tau = np.sign(q[0]) * tau_limit
        else:
            kq = 200
            kv0 = 0
            kv1 = 5
            # tau = kq * (q[0] + 0.55 * q[1])
            tau = np.dot(-self.K, x)
            # tau += - kv0 * v[0] + kv1 * (v[0] + v[1])

            # tauG = m1 * g * lc1 * sin(q[0]) + m2 * g * cos(q[0] + q[1]) * l1 * -sin(q[1]) + m2 * v[1]**2 / lc2 * l1 * sin(q[1])
            tauG = np.linalg.lstsq(M, tauG - Cv)[0][0] / np.linalg.lstsq(M, B[:,0])[0][0]
            # tau += -tauG
            if (tau + tauG) * q[0] > 0:
                tau = -tauG

        return tau

    def half_controller(self, x):
        q = x[:2]
        v = x[-2:]
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        x = np.array(x)
        x[0] -= np.pi
        x[:2] = mod_angle(x[:2])
        q = x[:2]
        v = x[-2:]

        tau_limit = 50
        kwall = 400
        ke = 0.5

        xf = np.array(x)
        # xf[1] = x[0] + x[1]
        # xf[3] = (1 + (l1 * m2 * lc2 * cos(x[1])) / (m2 * lc2**2)) * x[2] + x[3]
        xf[3] = 3./2 * x[2] + x[3]
        xf[0] += np.pi
        xf[2] = 0

        E_top = self.E(np.array([np.pi,0,0,0]))
        E_curl = self.E(xf) - E_top

        thresh = 0.5
        if E_curl > 0 and v[0] * v[1] > 0:
            tau = -kwall * np.sign(v[0])
        elif abs(q[0]) > thresh and q[0] * v[0] > 0:
            tau = -kwall * np.sign(q[0]) * (abs(q[0]) - thresh)
        else:
            ddv0_dtdu, ddv1_dtdu = np.linalg.lstsq(M, B[:,0])[0]
            # tau = ke * np.sign(cos(np.pi+q[0] + q[1])) * (v[0] + v[1]) * E_curl
            # tau = ke * np.sign(v[0] + v[1]) * E_curl
            a0, a1 = np.linalg.lstsq(M, -Cv + tauG)[0]
            tau = ke * np.sign(a0 + a1) * E_curl
            tau += max(0, E_curl/E_top - 0.8) * np.dot(-self.K_half, np.array([q[0], v[0]]))
            if tau * q[0] > 0:
                tau *= max(0, thresh - abs(q[0]))
            # print(x, E_curl, tau, xf)
        # print(x, E_curl, tau)

        tauG = np.linalg.lstsq(M, tauG - Cv)[0][0] / np.linalg.lstsq(M, B[:,0])[0][0]
        tau += -tauG

        return tau

    def E(self, state):
        q = state[:2]
        v = state[-2:]
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        theta_4 = m1 * lc1 + m2 * l1
        theta_5 = m2 * lc2
        E = 1./2 * np.dot(v.T, np.dot(M, v)) + theta_4 * g * -cos(q[0]) + theta_5 * g * -cos(q[0]+q[1])

        return E

    def ddE_dtdu(self, state):
        q = state[:2]
        v = state[-2:]
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)
        ddv_dtdu = np.linalg.lstsq(M, B[:,0])[0]
        ddE_dtdu = 1./2 * (np.dot(ddv_dtdu.T, np.dot(M, v)) + np.dot(v.T, B[:,0]))
        return ddE_dtdu

    def swingup_controller(self, state):
        q = state[:2]
        v = state[-2:]
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        E = self.E(state)
        E_top = self.E(np.array([np.pi, 0, 0, 0]))
        E_curl = E - E_top

        ddE_dtdu = self.ddE_dtdu(state)
        if ddE_dtdu == 0:
            ddE_dtdu = 1

        # state2 = np.array(state)
        # state2[3] = 0
        # E2 = self.E(state2)
        # E2_curl = E - E2

        ddv0_dtdu, ddv1_dtdu = np.linalg.lstsq(M, B[:,0])[0]
        # if ddv0_dtdu == 0:
            # ddv0_dtdu = 1
        # if ddv1_dtdu == 0:
            # ddv1_dtdu = 1
        q0bar = mod_angle(q[0] - np.pi)

        K_E = 0.5
        K_q0 = 0.0005
        K_v0 = 0.01
        K_v1 = 0.8

        tau = -K_E * E_curl * ddE_dtdu - (K_q0 * np.sin(q0bar)*np.exp(-q0bar**2) + K_v0 * v[0]) * ddv0_dtdu - K_v1 * v[1] * ddv1_dtdu

        max_tau = 10
        tau = np.clip(tau, -max_tau, max_tau)
        # print(tau, q, v, E, ddE_dtdu, ddv0_dtdu, ddv1_dtdu)
        return tau

    def _GetTorque(self, state):
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        # Extract manipulator dynamics.
        q = mod_angle(state[:2])
        v = state[-2:]

        # if self.calcV(state) < 10:
        q0bar = mod_angle(q[0] - np.pi)
        self.lqr_cond |= self.calcV(state) < 100
        # lqr_cond = abs(q0bar) < 1 and abs(q0bar+q[1]) < 0.5 and abs(v[0] + v[1]) < 1
        # lqr_cond = False
        # extend_cond = False
        # half_cond = abs(q0bar) < 1
        self.half_cond |= self.calcV_half(state) < 10
        # half_cond = True
        if self.lqr_cond:
            tau = self.lqr_controller(state)
        elif self.extend_cond:
            tau = self.extend_controller(state)
        elif self.half_cond:
            tau = self.half_controller(state)
        else:
            tau = self.swingup_controller(state)
        # print(state, tau, lqr_cond, half_cond)

        tau_limit = 50
        tau = np.clip(tau, -tau_limit, tau_limit)
        # print(q, v, self.lqr_cond, self.half_cond, tau)
        return tau

    def calcV(self, x):
        x = np.array(x)
        x[0] -= np.pi
        x[:2] = mod_angle(x[:2])
        return np.dot(x.T, np.dot(self.S, x)).item()

    def calcV_half(self, x):
        x = np.array(x[::2])
        x[0] -= np.pi
        x[0] = mod_angle(x[0])
        return np.dot(x.T, np.dot(self.S_half, x)).item()

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

    def xin_controller(self, state):
        '''
        From Xin et al. Broken implementation.
        '''
        g, m1, l1, lc1, m2, l2, lc2 = self.g, self.m1, self.l1, self.lc1, self.m2, self.l2, self.lc2
        q = state[:2]
        v = state[-2:]

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

        condition =  abs(E_curl) < min(2 * theta_4 * g, 2 * theta_5 * g, k_d / (k_e * theta_1))

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

        return tau

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
