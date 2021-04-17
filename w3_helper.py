#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 17 13:39:39 2021

@author: nigel
"""

import time
try:
    from  ezblock  import *
    from ezblock import __reset_mcu__
    __reset_mcu__()
    time.sleep (0.01)
except  ImportError:
    print ("This  computer  does  not  appear  to be a PiCar -X system(/opt/ezblock  is not  present). Shadowing  hardware  calls with  substitute  functions ")
    from  sim_ezblock  import *
from picarx_w3 import picar_thing
class sensor():
    def __init__(self):
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

    def get_adc_value(self): # finds the values going to the ADC pins
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list
    
class interpreter():
    def __init__(self,sensitivity=0.5,polarity=1):
        if type(sensitivity) is not float:
            print('sensitivity should be a float, using 0.5')
            self.sensitivity=0.5
        else:
            self.sensitivity=sensitivity
        #polarity = 1 means line is brighter than backdrop, -1 means opposite
        if polarity > 0:
            self.polarity=1
        else:
            self.polarity=-1
        self.past_values = []
        
    def process(self,sensor_value):
        self.past_values.append(sensor_value)
        if len(self.past_values)>5:
            self.past_values.pop(0)
        pos=0
        if (self.polarity*(sensor_value[1]-sensor_value[0]))>self.sensitivity:
            print('minor edge on left side')
            pos=-0.5
        elif (self.polarity*(sensor_value[1]-sensor_value[2]))>self.sensitivity:
            print('minor edge on right side')
            pos=0.5
        elif (self.polarity*(sensor_value[1]-sensor_value[0]))>self.sensitivity:
            print('major edge on left side')
            pos=-1
        elif (self.polarity*(sensor_value[1]-sensor_value[2]))>self.sensitivity:
            print('major edge on right side')
            pos=1
        return pos
        
class controller():
    def __init__(self,scaling_factor=1):
        self.scaling_factor = 1
        self.car = picar_thing()
        
    def controll_car(self,pos):
        self.car.forward(20,pos*self.scaling_factor)
        time.sleep(0.2)
        
def follow_line(*args):
    if len(args)==3:   
        car_sensor=sensor()
        car_interpret=interpreter(args[0],args[1])
        car_controll=controller(args[2])
    else:
        car_sensor=sensor()
        car_interpret=interpreter()
        car_controll=controller()
    while True:
        brightness=car_sensor.get_adc_value()
        pose=car_interpret.process(brightness)
        car_controll.controll_car(pose)