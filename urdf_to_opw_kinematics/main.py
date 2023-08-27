#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm
from urdf_to_opw_kinematics.util import angle, Axis, distance, rot_y, transform
from urdf_parser_py import urdf

DEBUG = True


def check_compatibility(robot: urdf.Robot, base_link: str = 'base_link', tip_link: str = 'tool0') -> bool:
    """ TODO add compatibility tests
    now I just check if there are 6 revolute joints
    """
    axes = get_joint_axes_from_urdf(robot, base_link)
    num_joints = len(axes)
    if num_joints != 6:
        print(robot.name + " has " + str(num_joints) + " joints, not 6.")
        return False
    return True


def convert(robot: urdf.Robot, base_link: str = 'base_link', tip_link: str = 'tool0') -> dict:
    axes = get_joint_axes_from_urdf(robot, base_link, tip_link)
    tool0_position = get_tool0_position(robot, axes, tip_link)

    jo = get_joint_offsets(axes)
    sc = get_sign_corrections(axes)

    params = get_dimensions(axes, tool0_position, jo)
    params['joint_offsets'] = jo
    params['sign_corrections'] = sc
    return params


def get_joint_axes_from_urdf(robot: urdf.Robot, base_link: str = 'base_link', tip_link: str = 'tool0') -> [Axis]:
    """ Extract joint origin and axis direction from urdf

    Save absolute position in base_link, and relative position
    with respect to the previous link.

    Parameters
    ----------
    robot object from the urdf_parser_py library

    Returns
    -------
    list of Axis objects
    """
    axes = []

    joints = []
    # Work upwards in joint-tree
    (joint_name, parent_name) = robot.parent_map[tip_link]
    while True:
        # prepend
        joints.insert(0, robot.joint_map[joint_name])
        if parent_name == base_link:
            break

        (joint_name, parent_name) = robot.parent_map[parent_name]

    # joints = robot.joints
    for joint in joints:
        # Find the joint corresponding to base_link
        print(joint.name)
        if len(axes) == 0:
            if joint.parent == base_link:
                axes.append(Axis(transform(joint), np.array(joint.axis)))
                # axes.append(Axis(np.array(joint.origin.xyz), np.array(
                #     joint.origin.xyz), np.array(joint.axis)))
                print(axes[-1])
            # else continue
        else:
            if joint.type == 'revolute':
                # p_relative = np.array(joint.origin.xyz)
                # p_previous = axes[-1].position
                t_previous = axes[-1].transform
                t_relative = transform(joint)
                axes.append(Axis(np.dot(t_previous, t_relative), np.array(joint.axis)))
                # axes.append(Axis(p_previous + p_relative,
                #                  p_relative, np.array(joint.axis)))
                print(axes[-1])
    return axes


def get_tool0_position(robot: urdf.Robot, axes: [Axis], tip_link: str = 'tool0') -> np.array:
    """ Search for the tool0 link and get absolut position origin

    Returns
    -------
    absolute position as a numpy array of length 3
    """
    for joint in robot.joints:
        if joint.child == tip_link:
            return axes[-1].position + np.array(joint.origin.xyz)
    raise ValueError("Failed to find a joint with child link 'tool0'.")


def get_joint_offsets(axes: [Axis]) -> [float]:
    """ Calculate joint angle difference between reference pose of opw_kinematics
    and the zero pose of the current robot
    """
    G1 = axes[0]
    G2 = axes[1]
    G3 = axes[2]
    G4 = axes[3]
    G5 = axes[4]
    G6 = axes[5]

    unit_x = np.array([1.0, 0, 0])
    unit_y = np.array([0, 1.0, 0])
    unit_z = np.array([0, 0, 1.0])

    jo1 = angle(unit_y, G2.direction)
    v23 = distance(G2, G3, return_vector=True)
    jo2 = angle(unit_z, v23)
    # g4_positive = np.array([abs(e) for e in axes[3].direction])
    jo3 = angle(unit_z, G4.direction) - jo2
    jo4 = angle(unit_y, G5.direction) - jo1
    jo5 = angle(G4.direction, G6.direction)

    # TODO get ee_y as input and correct for jo1 and j04
    ee_y_direction = unit_y
    jo6 = angle(ee_y_direction, unit_y)

    return [-jo1, -jo2, -jo3, -jo4, -jo5, -jo6]


def get_sign_corrections(axes: [Axis]) -> [int]:
    """ Does the axis rotate according to the right hand rule?
    Assume all z-axis pointed up and axis along one of the main axes
    """
    sc = map(np.sum, [a.direction for a in axes])
    return [int(val) for val in sc]


def get_dimensions(axes: [Axis], tool0_position: np.array, jo: [float]) -> dict:
    """ Calculate distance parameters c1, c2, c3, c4
    and signed distances a1, a2, b

    Note
    ----
    The sign of b is not yet implemented and defaults as positive
    """
    params = {}
    G1 = axes[0]
    G2 = axes[1]
    G3 = axes[2]
    G4 = axes[3]
    G5 = axes[4]
    G6 = axes[5]
    p_ee = tool0_position
    unit_x = np.array([1.0, 0, 0])

    # TODO use joint offset on first joint to make this more general
    # check if a1 is along positive x and g2 is above x-y plane
    # this mean that the position of g2 position should be (a1, 0, c1) with a1 > 0
    P2 = G2.position
    if (P2[0] >= 0 and P2[1] == 0 and P2[2] >= 0):
        params['a1'] = P2[0]
        params['c1'] = P2[2]
    else:
        raise ValueError("Wrong orientations of g2.")

    # c's are always positive
    params['c2'] = distance(G2, G3)
    params['c3'] = distance(G3, G5, along=G4)
    # distance between g5 and tool0 along g6
    params['c4'] = np.abs(np.dot(G6.direction, p_ee - G5.position))

    # calculate sign a2
    v34 = distance(G3, G4, return_vector=True)
    v34 = np.dot(rot_y(jo[1] + jo[2]), v34)
    ux_a2 = np.dot(G3.transform[:3, :3], unit_x)
    a2_sign = np.sign(np.dot(ux_a2, v34))

    params['a2'] = a2_sign * distance(G3, G4)

    # TODO sign calculation
    # but b is zero in most robots
    params['b'] = distance(G3, G4, along=G3)

    return params


def get_dimensions_new(axes: [Axis]) -> dict:
    """ (DOES NOT WORK)
    Alternative method that could work if we make a lot more assumtions about
    the given urdf model.
    """
    params = {}
    P_0_1 = axes[0].p_rel
    P_1_2 = axes[1].p_rel
    P_2_3 = axes[2].p_rel
    P_3_4 = axes[3].p_rel
    P_4_5 = axes[4].p_rel
    P_5_6 = axes[5].p_rel

    params['c1'] = P_0_1[2] + P_1_2[2]
    params['c2'] = norm(P_2_3)
    params['c3'] = norm(P_4_5) + P_3_4[0]
    params['c4'] = norm(P_5_6)

    params['a1'] = np.sqrt(P_1_2[0]**2 + P_1_2[1]**2)
    params['a2'] = -np.sqrt(P_3_4[0]**2 + P_3_4[2]**2)  # or -P_3_4[2]
    params['b'] = P_3_4[1]
    return params
