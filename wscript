#! /usr/bin/env python
#| This file is a part of the sferes2 framework.
#| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
#| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
#|
#| This software is a computer program whose purpose is to facilitate
#| experiments in evolutionary computation and evolutionary robotics.
#| 
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#| 
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.
import sys
import os
import sferes
sys.path.insert(0, sys.path[0]+'/waf_tools')
print sys.path[0]
#from waflib.Build import BuildContext

from waflib.Configure import conf

import dart
import hexapod_controller



def options(opt) : 
    opt.load('dart')
    opt.load('hexapod_controller')
    opt.load('hexapod_dart_simu')


@conf
def configure(conf): 
    print 'conf exp:'
    conf.load('dart')
    conf.load('hexapod_controller')
    conf.load('hexapod_dart_simu')
    conf.check_dart()
    conf.check_hexapod_controller()
    conf.check_hexapod_dart_simu()


def build(bld):

    libs = 'DART  EIGEN BOOST BOOST_UNIT_TEST_FRAMEWORK  BOOST_TIMER TBB HEXAPOD_DART_SIMU HEXAPOD_CONTROLLER'
    
    print ("Entering directory `" + os.getcwd() + "/modules/'")
    #test_map_elite = bld.new_task_gen('cxx', 'program')
    #test_map_elite.source = 'test_map_elite.cpp'
    #test_map_elite.includes = '. ../../'
    #test_map_elite.uselib_local = 'sferes2'
    #test_map_elite.uselib = libs
    #test_map_elite.target = 'test_map_elite'
    #test_map_elite.unit_test = 1


    bld.program(features='cxx cxxstlib',
                source='arm_hori.cpp',
                includes='. .. ../../',
                target='robot',
                uselib=libs)
                
    sferes.create_variants(bld,
                           source = 'scenario_arm.cpp',
                           use = 'sferes2 robot',
                           uselib = libs,
                           target = 'scenario_arm',
                           json = '',
                           variants = ['GRID RANDOM',   'GRID FITNESS',   'GRID NOVELTY',   'GRID CURIOSITY',   'GRID NOSELECTION',   'GRID POPFITNESS',   'GRID POPNOVELTY',   'GRID POPCURIOSITY', 'GRID PARETO', 
                                       'ARCHIVE RANDOM','ARCHIVE FITNESS','ARCHIVE NOVELTY','ARCHIVE CURIOSITY','ARCHIVE NOSELECTION','ARCHIVE POPFITNESS','ARCHIVE POPNOVELTY','ARCHIVE POPCURIOSITY', 'ARCHIVE PARETO'])

    sferes.create_variants(bld,
                           source = 'scenario_hexa_turn.cpp',
                           use = 'sferes2',
                           uselib = libs,
                           target = 'scenario_hexa_turn',
                           json = '',
                           variants = ['GRID RANDOM',   'GRID FITNESS',   'GRID NOVELTY',   'GRID CURIOSITY',   'GRID NOSELECTION',   'GRID POPFITNESS',   'GRID POPNOVELTY',   'GRID POPCURIOSITY', 
                                       'ARCHIVE RANDOM','ARCHIVE FITNESS','ARCHIVE NOVELTY','ARCHIVE CURIOSITY','ARCHIVE NOSELECTION','ARCHIVE POPFITNESS','ARCHIVE POPNOVELTY','ARCHIVE POPCURIOSITY'])




    #scenario_arm = bld.new_task_gen('cxx', 'program')
    #scenario_arm.source = 'scenario_arm.cpp arm_hori.cpp'
    #scenario_arm.includes = '. ../../'
    #scenario_arm.uselib_local = 'sferes2'
    #scenario_arm.uselib = 'EIGEN3 BOOST BOOST_UNIT_TEST_FRAMEWORK  BOOST_TIMER TBB'
    #scenario_arm.target = 'scenario_arm'
    #scenario_arm.variants = ['RANDOM', 'FITNESS', 'NOVELTY', 'CURIOSITY']
