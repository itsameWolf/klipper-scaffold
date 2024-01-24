# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper, mathutil, chelper

class PolarCraneKinematics:
    def __init__(self, toolhead, config):
        
        # Setup axis steppers
        rail_column = stepper.PrinterRail(config.getsection('stepper_column'), 
                                          units_in_radians=True)
        rail_arm = stepper.PrinterRail(config.getsection('stepper_arm'))
        
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        
        rail_column.setup_itersolve('polar_stepper_alloc', b'a')
        rail_arm.setup_itersolve('polar_stepper_alloc', b'r')
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        
        self.rails = [rail_column, rail_arm, rail_z]
        self.steppers = [ s for r in self.rails
                         for s in r.get_steppers() ]
        
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        max_xy = self.rails[1].get_range()[1]
        min_z, max_z = self.rails[2].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.)
        
        # Homing trickery fake cartesial kinematic
        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cartesian_kinematics_COL = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        self.cartesian_kinematics_ARM = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'y'), ffi_lib.free)
        
    def get_steppers(self):
        return list(self.steppers)
    
    def get_polar_steppers(self):
        return list(self.steppers[0:2])
    
    def calc_position(self, stepper_positions):
        column_angle = stepper_positions[self.rails[0].get_name()]
        arm_pos = stepper_positions[self.rails[1].get_name()]
        z_pos = stepper_positions[self.rails[2].get_name()]
        return [math.cos(column_angle) * arm_pos, math.sin(column_angle) * arm_pos,
                z_pos]
        
    def polar_to_cart(self, angle, radius):
        return [math.cos(angle) * radius, math.sin(angle) * radius]
    
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[2].get_range()
        if 0 in homing_axes and 1 in homing_axes:
            self.limit_xy2 = self.rails[1].get_range()[1]**2
            
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)
    
    def home(self, homing_state):
        # Always home XY together
        homing_axes = homing_state.get_axes()
        home_x = 0 in homing_axes 
        home_y = 1 in homing_axes
        home_z = 2 in homing_axes
        

        if home_x:
            homing_state.set_axes([0])
            rails = [self.rails[0], self.rails[1]]
            
            col_endstop = rails[0].get_homing_info().position_endstop
            col_min, col_max = rails[0].get_range()
            
            arm_endstop = rails[1].get_homing_info().position_endstop
            arm_min, arm_max = rails[1].get_range()
            
            # Swap to linear kinematics
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            
            steppers = self.get_polar_steppers()
            
            kinematics = [self.cartesian_kinematics_COL,
                          self.cartesian_kinematics_ARM]
            prev_sks    = [stepper.set_stepper_kinematics(kinematic)
                            for stepper, kinematic in zip(steppers, kinematics)]
            
            try:
                homepos  = [col_endstop, arm_endstop, None, None]
                hil = rails[0].get_homing_info()
                if hil.positive_dir:
                    forcepos = [0, 0, None, None]
                else:
                    forcepos = [col_max, 0, None, None]
                
                homing_state.home_rails([rails[0]], forcepos, homepos)

                for stepper, prev_sk in zip(steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)

                [x,y] = self.polar_to_cart(
                    rails[0].get_homing_info().position_endstop,
                    rails[1].get_homing_info().position_endstop)
                    
                toolhead.set_position( [x, y, 0, 0], (0, 1))
                toolhead.flush_step_generation()

            except Exception as e:
                for stepper, prev_sk in zip(steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)
                toolhead.flush_step_generation()
                raise
            
        if home_y:
            homing_state.set_axes([0, 1])
            rails = [self.rails[0], self.rails[1]]
            
            col_endstop = rails[0].get_homing_info().position_endstop
            col_min, col_max = rails[0].get_range()

            arm_endstop = rails[1].get_homing_info().position_endstop
            arm_min, arm_max = rails[1].get_range()
            
            # Swap to linear kinematics
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            
            steppers = self.get_polar_steppers()
            
            kinematics = [self.cartesian_kinematics_COL,
                          self.cartesian_kinematics_ARM]
            prev_sks    = [stepper.set_stepper_kinematics(kinematic)
                            for stepper, kinematic in zip(steppers, kinematics)]
            
            try:
                homepos  = [col_endstop, arm_endstop, None, None]
                hil = rails[0].get_homing_info()
                if hil.positive_dir:
                    forcepos = [0, 0, None, None]
                else:
                    forcepos = [0, arm_max, None, None]
                
                homing_state.home_rails([rails[1]], forcepos, homepos)

                for stepper, prev_sk in zip(steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)

                [x,y] = self.polar_to_cart(
                    rails[0].get_homing_info().position_endstop,
                    rails[1].get_homing_info().position_endstop)
                    
                toolhead.set_position( [x, y, 0, 0], (0, 1))
                toolhead.flush_step_generation()

            except Exception as e:
                for stepper, prev_sk in zip(steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)
                toolhead.flush_step_generation()
                raise
            
        if home_z:
            rail = self.rails[2]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None]
            homepos[2] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[2] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[2] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)
            
    def _motor_off(self, print_time):
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        
    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0]**2 + end_pos[1]**2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            
    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PolarCraneKinematics(toolhead, config)
