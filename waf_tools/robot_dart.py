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
from waflib.Tools import waf_unit_test

def options(opt):
    opt.load('compiler_cxx')
    opt.load('compiler_c')
    # opt.load('boost')
    opt.load('dart')
    opt.add_option('--robot_dart', type='string', help='path to robot_dart', dest='robot_dart')

@conf
def check_robot_dart(conf):
    # conf.load('boost')
    # conf.load('eigen')
    # conf.load('dart')
    # conf.load('avx')

    # conf.check_boost(lib='regex system filesystem unit_test_framework', min_version='1.46')
    conf.check_dart()

    includes_check = ['/usr/local/include/robot_dart', '/usr/include/robot_dart']
    libs_check = ['/usr/local/lib', '/usr/lib']

    if conf.options.robot_dart:
        includes_check = [conf.options.robot_dart + '/include/robot_dart']
        libs_check = [conf.options.robot_dart + '/lib']
    try:
        conf.start_msg('Checking for robot_dart includes')
        res = conf.find_file('robot.hpp', includes_check)
        res = res and conf.find_file('robot_dart_simu.hpp', includes_check)
        res = res and conf.find_file('utils.hpp', includes_check)
        res = res and conf.find_file('control/pid_control.hpp', includes_check)
        res = res and conf.find_file('descriptor/base_descriptor.hpp', includes_check)
        res = res and conf.find_file('graphics/base_graphics.hpp', includes_check)
        res = res and conf.find_file('graphics/camera_osr.hpp', includes_check)
        res = res and conf.find_file('graphics/graphics.hpp', includes_check)
        res = res and conf.find_file('graphics/pbuffer_manager.hpp', includes_check)
        res = res and conf.find_file('arm/arm_simulation.hpp', includes_check)
        conf.end_msg('ok')
        conf.start_msg('Checking for robot dart lib')
        res = res and conf.find_file('libRobotDARTSimu.a', libs_check)
        conf.end_msg('ok')
        conf.env.INCLUDES_ROBOT_DART = includes_check
        conf.env.LIBPATH_ROBOT_DART = libs_check
        conf.env.DEFINES_ROBOT_DART = ['GRAPHIC']
        conf.env.LIB_ROBOT_DART = ['RobotDARTSimu']
    except:
        conf.end_msg('Not found', 'RED')
        return
    return 1
