#!/usr/bin/env python
from urdf_parser_py.urdf import Robot
from ament_index_python import get_package_share_directory

from urdf_to_opw_kinematics.main import convert
import sys

PI = 3.14159265359

params_kr6 = {
    "c1":  0.400,
    "c2":  0.315,
    "c3":  0.365,
    "c4":  0.080,
    "a1":  0.025,
    "a2": -0.035,
    "b":   0.000,
    "joint_offsets": [0, -PI / 2, 0, 0, 0, 0],
    "sign_corrections": [-1, 1, 1, -1, 1, -1]
}


def print_params(params: dict) -> None:
    # Print parameters in the same order
    keys = ['a1', 'a2', 'b', 'c1', 'c2', 'c3',
            'c4', 'joint_offsets', 'sign_corrections']
    for key in keys:
        print("{:17} {}".format(key + ':', str(params[key])))


def run(filename: str, base_link: str, tip_link: str):
    print(filename)
    with open(filename, 'rb') as urdf_file:
        urdf_str = urdf_file.read()
        robot = Robot.from_xml_string(urdf_str)
        params = convert(robot, base_link, tip_link)

        print('Calculated values from ' + filename + ':')
        print_params(params)


def example():
    filename = get_package_share_directory(
        'urdf_to_opw_kinematics') + '/urdf/kr6r700.urdf'
    run(filename, 'base_link', 'tool0')
    print()
    print('They should be:')
    print_params(params_kr6)


def main():
    argc = len(sys.argv)
    print(sys.argv, argc)
    if argc < 2:
        print('Supply a urdf file as argument')
        return
    if argc < 4:
        print("""
Optionally supply 'base_link' and 'tip_link' as params"
example: ros2 run urdf_to_opw_kinematics run <urdf_path> <base_link> <tool0>
Trying to use 'base_link' and 'tool0'
        """)

    run(sys.argv[1],
        sys.argv[2] if argc > 3 else 'base_link',
        sys.argv[3] if argc > 3 else 'tool0'
        )


if __name__ == "__main__":
    example()
