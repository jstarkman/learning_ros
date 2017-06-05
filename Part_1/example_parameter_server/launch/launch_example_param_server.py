#!/usr/bin/env python3

# Original:
# <launch>
#     <rosparam command="load" file="$(find example_parameter_server)/launch/jnt1_gains.yaml" />
# </launch>

import os
import sys
from tempfile import NamedTemporaryFile

import yaml

from launch import LaunchDescriptor
from launch.exit_handler import ignore_exit_handler
from launch.exit_handler import restart_exit_handler
from launch.loader import load_launch_file
from launch.output_handler import FileOutput
from launch.output_handler import ConsoleOutput

def mkparam(blob, prefix=""):
    if type(blob) == dict:
        out = []
        for k, v in blob.items():
            out.extend(mkparam(v, prefix + "/" + k))
        return out
    else:
        return [prefix + " " + str(blob)]

def launch(launch_descriptor, argv):
    executable_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'read_param_from_node')
    gains_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'jnt1_gains.yaml')
    
    ld = launch_descriptor
    
    ld.add_process(
        cmd=["paramsrv"],
        name='paramsrv',
        exit_handler=restart_exit_handler,
    )

    with open(gains_file, "r") as stream:
        params = yaml.load(stream)
        calls = mkparam(params, prefix="paramsrv") # node name
        for call in calls:
            param, value = call.split()
            ld.add_process(
                cmd=["ros2param", "set", param, value],
                name='tmp' + str(call),
                env=None,
                exit_handler=ignore_exit_handler,
            )
