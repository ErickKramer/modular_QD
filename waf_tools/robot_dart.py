#! /usr/bin/env python
# encoding: utf-8
# Erick Kramer- 2019

"""
Quick n dirty robot_dart detection
"""

import os
from waflib.Configure import conf


def options(opt):
    opt.add_option('--robot_dart', type='string', help='path to robot_dart', dest='robot_dart')

@conf
def check_robot_dart(conf):
    includes_check = ['/usr/local/include', '/usr/include']

    if conf.options.robot_dart:
        includes_check = [conf.options.robot_dart + '/include']
    try:
    	conf.start_msg('Checking for robot_dart includes')
    	res = conf.find_file('robot_dart/robot.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/robot_dart_simu.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/utils.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/control/pid_control.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/descriptor/base_descriptor.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/graphics/base_graphics.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/graphics/camera_osr.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/graphics/graphics.hpp', includes_check)
    	res = res and conf.find_file('robot_dart/graphics/pbuffer_manager.hpp', includes_check)
    	conf.end_msg('ok')
    	conf.env.INCLUDES_ROBOT_DART = includes_check
    except:
    	conf.end_msg('Not found', 'RED')
    	return
    return 1
