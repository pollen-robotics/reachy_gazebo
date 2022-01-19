#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  File Name	: set_param.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	: Thursday, January  6 2022
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#


import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, SetParametersResult, ParameterType
from rclpy import parameter
import yaml
import argparse
import time
# from threading import Thread


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


class ParameterManager(Node):
    def __init__(self, param_dict):
        super().__init__("dummy_parameter_manager_node")
        self._param_dict = param_dict

        # this seems to be very unreliable...
        # self.node_list = self.get_node_names_and_namespaces()

        self.node_list = []
        for p, v in self._param_dict.items():
            self.node_list.append(p)

        # print(f'NODES: {self.node_list}')

        self.futures = {}
        self.client_dict = {}

        time.sleep(3)
        self.get_logger().warn(f'Setting parametersa')

        self.get_logger().info(f'NODES: {self.node_list}')

        for n in self.node_list:
            if (not "dummy_parameter_manager_node" in n) and (not "ros2cli_daemon" in n):

                c = self.create_client(
                    SetParameters, n+'/set_parameters')
                self.client_dict[n] = c
                c.wait_for_service()  # sometimes freezes...

        # print("Wait ok")
        self.get_logger().info(f'Wait ok')

        self.set_from_yaml()

    def set_from_yaml(self):
        '''
        cannot handle the 'namespace' yet...
        So assuming this format:

        node_name:
          ros__parameters:
            param_name: param_value

        '''
        for p, v in self._param_dict.items():

            node_name = p
            params = v['ros__parameters']
            params_list = []
            for param_name, param_value in params.items():
                # print(f'TYPE: {param_name} {type(param_value)}')
                param = parameter.Parameter(
                    name=param_name, value=param_value).to_parameter_msg()
                params_list.append(param)
            if len(params_list) > 0:
                req = SetParameters.Request()
                req.parameters = params_list

            # print(f'TEST: {p} {self.client_dict}')
            if p in self.client_dict:
                # print(f'setting: {p} {params_list}')
                self.get_logger().info(f'Setting: {p} {v}')

                self.futures[p] = self.client_dict[p].call_async(req)
            else:  # should not happen
                # print(f'node does not exist')
                self.get_logger().info(f'Node {p} does not exist')

        # print('YAML done')
        self.get_logger().info(f'YAML done')


def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument('yaml',
                        help='parameter yaml')

    # parser.add_argument('--ros-args',
    #                     help='Ignored')

    # parser.add_argument('--params-file',
    #                     help='Ignored')

    parsed_args, unknown = parser.parse_known_args()

    # parsed_args = parser.parse_args()
    params = None
    try:
        with open(parsed_args.yaml, 'r') as file:
            params = yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        print(f'Cannot open file: {parsed_args.yaml}')
        return None

    # print(params)

    rclpy.init(args=args)

    parammanager = ParameterManager(params)

    while rclpy.ok():

        rclpy.spin_once(parammanager)
        incomplete_futures = {}
        for fname, future in parammanager.futures.items():
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    parammanager.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    if not response.results[0].successful:

                        parammanager.get_logger().info(
                            f'Result {response} {fname} {future}')
                    else:
                        print(f'{fname} set')

            else:
                incomplete_futures[fname] = future
                parammanager.futures = incomplete_futures

        if len(incomplete_futures) == 0:
            break
        else:
            for n, f in incomplete_futures.items():
                # print(f'\t missing: {n}')
                parammanager.get_logger().info(
                    f'\t missing: {n}')

    parammanager.get_logger().warn(f'All set')
    parammanager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
