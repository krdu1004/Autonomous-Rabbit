import cv2
import numpy as np
import math
import statistics 
import time
from matplotlib import pyplot as plt
from GBController import GBController
from controller_timer import check_timer, start_timer, start_time

def acceleration_phase(t_i, v_i, v_d, k, s):

    '''
    t: time
    t_i: initial time
    v_i: initial velocity
    v_d: desired velocity
    k: sigmoid steepness parameter
    s: sigmoid shift parameter

    '''
    t = time.time()

    sigmoid = 1 / (1 + np.exp(-k * ((t - t_i) - s)))

    v_sigmoid = v_i + (v_d - v_i)*sigmoid

    return v_sigmoid

def get_current_optimal_speed(optimal_speed_pattern_schedule, runner_position):

    '''
    FOR ELITE
    Tar inn ferdig justert tabell for optimal hastighet og løperens posisjon.
    Gir ut hvilken hastighet [m/s] løperen/haren optimalt skal holde ut ifra posisjon.
    '''
    global j, Vd_APM_last, Vd_APM

    Vd_tab = optimal_speed_pattern_schedule['Speed(m/s)']
    d_segments = optimal_speed_pattern_schedule['Length']
    expedite_t = 7 #Expedite time of the acceleration 

    if j < Vd_tab.shape[0] and runner_position >= d_segments[j] - Vd_tab.at[j, 'Length']*expedite_t:
        Vd_APM_last = Vd_APM
        Vd_APM = Vd_tab.at[j+1, 'Length'] #Checking next segment speed.
        # Update counter
        j += 1 
    else:
        Vd_APM = Vd_tab.at[j, 'Length'] 
    return Vd_APM, j

def updateCCparameters(Vd_list, p_segment_list, total_pos, p_expedite, V_APM):


def cruise_controller(Vd_APM, Vi_APM, t_i_cc, k_a, s_a, k_d, s_d):
    
    if Vd_APM > Vi_APM:
        phase_var = 1
    elif Vd_APM < Vi_APM:
        phase_var = 3
    elif Vd_APM == 0:
        phase_var = 4
    else:
        phase_var = 2
    
    match phase_var:
        case 1: # First phase: acceleration
            cc_speed_ref = acceleration_phase(t_i_cc, Vi_APM, Vd_APM, k_a, s_a)
        case 2: # Second phase: constant velocity
            cc_speed_ref = Vd_APM
        case 3: # Third phase: deceleration
            cc_speed_ref = acceleration_phase(t_i_cc, Vi_APM, Vd_APM, k_d, s_d)
        case 4: # Fourth phase: constant positiion
            cc_speed_ref = 0
        case _:
            print('Error')

    return cc_speed_ref

def distance_controller(V_APM, runner_object, d_min, d_max, distance_Controller, setpoint_dc):
    # delta_p is the distance between the APM and the runner. From camera.


    runner_position = runner_object.position[2] #z-coordinate of the 3D position vector for detected object (delta_p in matlab code)

    distance_Controller.setpoint = setpoint_dc

    speed_change = distance_Controller.compute(runner_position)

    dc_speed_ref = V_APM + speed_change

    return dc_speed_ref

def PID_and_DC():
    '''
    Gives out value between 0-100 depending on precentage of max velocity
    Adaptive find out control paramteres of DC motor
    '''
    return 1

def check_within_bounds(runner_object, d_min, d_max):

    runner_pos = runner_object.position[2]

    if runner_pos > d_min & runner_pos < d_max:
        within_bounds = True

    else:
        within_bounds = False
        stable = False

        if runner_pos < d_min:
            d_ref = d_min
        elif runner_pos > d_max:
            d_ref = d_max

    
    stable = check_if_stable(d_ref, runner_pos)

    if (stable):
        d_ref = (d_min + d_max) / 2 # Inital distance refrence. d_ref = d_init. Should maybe be global

    return within_bounds, stable, d_ref

def check_if_stable(d_ref, runner_pos):
    
    # Overkill to send in stable, should maybe just set it to false inside this function? Since it will always be sent in as False

    t_shift = 2 # [s] Shift to stable if time excess t_shift
    delta = 0.01 # Allowed deviation from desired setpoint d_ref 

    stable = False
    
    if (not stable):

        # Check if the distance to the runner is sufficently close to the distance refrence 
        if (runner_pos > d_ref - delta) | (runner_pos < d_ref + delta): 

            # Start timer if not yet stared
            if start_time is None:
                start_timer()

            elif check_timer(t_shift):
                stable = True
                start_time = None
    
    return stable
    
def decide_control_method(runner_object, d_max, d_min, stable, d_ref, within_bounds):

    '''
    switch case for cc og dc 
    '''
    gamma = 1 # Must be initialized
    delta = 1 # [m] Allowed deviation of 1 meter
    prev_error = 0 # Unsure where this should be used

    d_init = (d_min + d_max) / 2 # Should be initalized before maybe as a global parameter?
    runner_pos = runner_object.position[2]

    # Cruise Control should be default, should control strategy maybe be global?
    if (within_bounds):
        if ((runner_pos > d_init - delta) | (runner_pos < d_init + delta)):
            # Shift to CC if stable and the distance to runner is satisfactory close to d_init
            control_strategy = 1
        else:
            control_strategy = 2
            prev_error = gamma*d_ref - runner_pos # Update error to PID Controller

    elif not within_bounds:  
        control_strategy = 2
        prev_error = gamma*d_ref - runner_pos # Update error to PID Controller



    return control_strategy, prev_error

"""
IN MAIN:
control_strategy, prev_error = decide_control_method(runner_object, d_max, d_min, within_bounds, stable, d_ref):

switch control_strategy
    case 1:
        v_ref = cruise_control()
    case 2:
        v_ref = distance_control()
"""