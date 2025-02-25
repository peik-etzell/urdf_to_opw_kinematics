#!/usr/bin/env python

import glob
import os
import sys
import yaml

# ros specific libraries
from urdf_parser_py.urdf import URDF  # nolint
from urdf_to_opw_kinematics.main import convert, check_compatibility

# PATTERN = "*_macro.xacro"
PATTERN = "*.urdf"


class RobotParser:
    """ Search for urdf models and extract
    the kinematic parameters
    """

    def __init__(self, command_line_arguments):
        file_paths = self.process_input(command_line_arguments)
        for fp in file_paths:

            robot = URDF.from_xml_file(fp)

            if check_compatibility(robot):
                p = convert(robot)
                self.write_params_to_yaml(p, fp)
            else:
                print(robot.name + " is not compatibel with opw_kinematics.")
                continue

    def process_input(self, argv):
        """ Check if a path is provided and find all robot description files

        The script should be used with rosrun!
        Rosrun passes a path as a first argument that we will ignore.
        The argument provided by the user will be the second element of argv.
        """
        if len(argv) == 1:
            print(
                "No path provided, looking for *.urdf files in current working directory.")
            start_dir = os.getcwd()
            print("cwd: " + start_dir)
        if len(argv) == 2:
            print("Thanks, I received a path, I hope it is an absolute one...")
            start_dir = argv[1]
            print(start_dir)
        if len(argv) > 2:
            print("To many command line arguments")
            print(
                "Expects maximum one, the absolute path where you want to look for robots.")
            exit(1)

        files = []
        for dir, _, _ in os.walk(start_dir):
            files.extend(glob.glob(os.path.join(dir, PATTERN)))

        if len(files) == 0:
            print(
                "I cannot find files named ****_macro.xacro, maybe specify a(nother) path?")
            exit(1)
        else:
            print("I will extract parameters from the following files:")
            for f in files:
                print("- " + os.path.basename(f))

        return files

    def remove_minus_sign_if_zero(self, val):
        if val == 0.0:
            return abs(val)
        else:
            return val

    def write_params_to_yaml(self, params, file_path):
        # get robot name from path and add extension
        name = os.path.basename(file_path)
        name = os.path.splitext(name)[0]
        name += ".yaml"

        # go thourgh all parameters that need type conversion
        # remove the minus sign from floats that are zero
        for key in params:
            if type(params[key]) is not list:
                params[key] = float(params[key])
                params[key] = self.remove_minus_sign_if_zero(params[key])
        params["joint_offsets"] = [float(angle)
                                   for angle in params["joint_offsets"]]
        params["joint_offsets"] = [self.remove_minus_sign_if_zero(
            angle) for angle in params["joint_offsets"]]

        # extend to contain all info for the moveit_config package
        data = {"manipulator": {}}
        data["manipulator"]["kinematics_solver"] = "moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin"
        data["manipulator"]["kinematics_solver_joint_offsets"] = params.pop(
            "joint_offsets")
        data["manipulator"]["kinematics_solver_joint_sign_corrections"] = params.pop(
            "sign_corrections")
        data["manipulator"]["kinematics_solver_geometric_parameters"] = {}
        data["manipulator"]["kinematics_solver_geometric_parameters"]['a1'] = params['a1']

        print(yaml.dump(data))
        with open(name, 'w') as outfile:
            yaml.dump(data, outfile)  # , default_flow_style=False)


def main():
    parser = RobotParser(sys.argv)


if __name__ == "__main__":
    main()
