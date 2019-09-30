#! /usr/bin/env python
# encoding: utf-8
# Erick Kramer- 2019

"""
Quick n dirty robot_dart detection
"""
import os
import boost
import eigen
import dart
from waflib.Configure import conf

def options(opt):
    opt.load('boost')
    opt.load('eigen')
    opt.load('dart')
    opt.add_option('--robot_dart', type='string', help='path to robot_dart', dest='robot_dart')

@conf
def check_robot_dart(conf):
    conf.load('boost')
    conf.load('eigen')
    conf.load('dart')

    conf.check_boost(lib='regex system filesystem unit_test_framework', min_version='1.46')
    conf.check_eigen()
    conf.check_dart()

    includes_check = ['/usr/local/include', '/usr/include']
    libs_check = ['/usr/local/lib', '/usr/lib']

    if conf.options.robot_dart:
        includes_check = [conf.options.robot_dart + '/include']
        libs_check = [conf.options.robot_dart + '/lib']
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
        res = res and conf.find_file('robot_dart/arm/arm_simulation.hpp', includes_check)
        conf.end_msg('ok')
        conf.env.INCLUDES_ROBOT_DART = includes_check
    except:
        conf.end_msg('Not found', 'RED')
        return
    return 1
