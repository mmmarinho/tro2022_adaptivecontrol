import numpy as np
from numpy import pi
from dqrobotics import *
from marinholab.papers.tro2022.adaptive_control import *

plot_enabled = None
try:
    import matplotlib.pyplot as plt
    import dqrobotics_extensions.pyplot as dqp
    plot_enabled = True
except ImportError:
    print("Running example without plot as the modules could not be found.")
    plot_enabled = False

def setup_plot():
    # Set up the plot
    fig = plt.figure()
    plot_size = 1
    ax = plt.axes(projection='3d')
    ax.set_xlabel('$x$')
    ax.set_xlim((-plot_size, plot_size))
    ax.set_ylabel('$y$')
    ax.set_ylim((-plot_size, plot_size))
    ax.set_zlabel('$z$')
    ax.set_zlim((-plot_size, plot_size))
    return fig, ax

def vs050_raw_kinematics() -> Example_SerialManipulatorEDH:
    """
    Gets the ideal kinematics of the VS050 robot.
    :return:
    """
    VS050_dh_matrix = np.array([
        [-pi, pi / 2, -pi / 2, 0, pi, 0],
        [0.345, 0, 0, 0.255, 0, 0.07],
        [0, 0.250, 0.01, 0, 0, 0],
        [pi / 2, 0, -pi / 2, pi / 2, pi / 2, 0],
        [0, 0, 0, 0, 0, 0]
    ])

    lower_joint_limits = np.array([-170, -100, -60, -265, -119, -355]) * pi / 180
    upper_joint_limits = np.array([170, 100, 124, 265, 89.9, 355]) * pi / 180

    robot = Example_SerialManipulatorEDH(VS050_dh_matrix)
    robot.set_lower_q_limit(lower_joint_limits)
    robot.set_upper_q_limit(upper_joint_limits)

    robot.set_base_frame(DQ([1]))
    robot.set_effector_frame(DQ([1]))

    return robot

def main():
    if plot_enabled: setup_plot()

    estimated_robot = vs050_raw_kinematics()
    q_init = np.array([0, pi/4, pi/2, 0, pi/4, 0])

    if plot_enabled:
        dqp.plot(estimated_robot,
                 q=q_init,
                 line_color='r',
                 cylinder_color="r",
                 cylinder_alpha=0.3)
        plt.show(block=True)


if __name__ == "__main__":
    main()