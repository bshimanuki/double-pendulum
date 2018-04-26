# -*- coding: utf-8 -*-

import argparse
from math import sin

from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyTree,
                         RigidBodyPlant, Simulator, SignalLogger, VectorSystem)
from underactuated import (FindResource, ManipulatorDynamics,
                           PlanarRigidBodyVisualizer)


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

    def _DoCalcVectorOutput(self, context, double_pend_state, unused, torque):
        # Extract manipulator dynamics.
        q = double_pend_state[:2]
        v = double_pend_state[-2:]
        (M, Cv, tauG, B) = ManipulatorDynamics(self.tree, q, v)

        # Desired pendulum parameters.
        length = 2.
        b = .1

        # Control gains for stabilizing the second joint.
        kp = 1
        kd = .1

        # Cancel double pend dynamics and inject single pend dynamics.
        torque[:] = Cv - tauG + \
            M.dot([-self.g / length * sin(q[0]) - b * v[0],
                   -kp * q[1] + kd * v[1]])


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
