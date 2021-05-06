#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  6 13:00:25 2021

@author: nigel
"""


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 25 13:17:27 2021

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
from threading import Lock
import concurrent.futures
from bus_structure import bus_message

class sensor():

    def __init__(self):
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')
        self.brightness_bus = bus_message()
        
    def get_adc_value(self,delay_time): # finds the values going to the ADC pins
        lock = Lock()

        while True:
            adc_value_list = []
            adc_value_list.append(self.S0.read())
            adc_value_list.append(self.S1.read())
            adc_value_list.append(self.S2.read())
            self.brightness_bus.write(adc_value_list)
            time.sleep(delay_time)
            global stop_threads
            if stop_threads:
                break
    
class interpreter():

    def __init__(self, sensitivity = 100, polarity = 1):

        if type(sensitivity) is not int:
            print('sensitivity should be an int, using 200')
            self.sensitivity = 100
        else:
            self.sensitivity = sensitivity
        #polarity = 1 means line is brighter than backdrop, -1 means opposite

        if polarity > 0:
            self.polarity = 1
        else:
            self.polarity = -1
        self.interpreter_bus = bus_message()
        
    def process(self,sensor_value_bus,delay_time):
        while True:
            global stop_threads
            sensor_value=sensor_value_bus.read()
            pos = 0
    
            if (self.polarity*(sensor_value[1] - sensor_value[0])) > self.sensitivity:
                pos = -0.5
            elif (self.polarity*(sensor_value[1] - sensor_value[2])) > self.sensitivity:
                pos = 0.5
            elif (self.polarity*(sensor_value[1] - sensor_value[0])) > self.sensitivity:
                pos = -1
            elif (self.polarity*(sensor_value[1] - sensor_value[2])) > self.sensitivity:
                pos = 1
            
            self.interpreter_bus.write(pos)
            time.sleep(delay_time)
            if stop_threads:
                break
        
class controller():

    def __init__(self, scaling_factor = 40):
        
        self.scaling_factor = scaling_factor
        self.car = picar_thing()
        
    def controll_car(self, pos_bus, delay_time):
        while True:
            global stop_threads
            pos = pos_bus.read()
            self.car.set_dir_servo_angle(pos*self.scaling_factor)
            self.car.forward(15, pos*self.scaling_factor)
            
            time.sleep(delay_time)
            if stop_threads:
                break

def end_threads():
    input('press enter to kill thing')
    global stop_threads
    stop_threads = True
    
def follow_line(*args):
    global stop_threads
    if len(args) == 3:   
        car_sensor = sensor()
        car_interpret = interpreter(args[0], args[1])
        car_controll = controller(args[2])
    else:
        car_sensor = sensor()
        car_interpret = interpreter()
        car_controll = controller()
    
    brightness_bus=car_sensor.brightness_bus
    pose_bus=car_interpret.interpreter_bus
    
    delay_time=0.5
    
    with concurrent.futures.ThreadPoolExecutor(max_workers = 4) as executor:
        eSensor = executor.submit(car_sensor.get_adc_value, delay_time)
        eInterpreter = executor.submit(car_interpret.process, brightness_bus, delay_time)
        eController = executor.submit(car_controll.controll_car, pose_bus, delay_time)
        eStop = executor.submit(end_threads)
        eStop.result()
        eSensor.result()
        eInterpreter.result()
        eController.result()
        
    '''
    print('making threads')
    t1 = threading.Thread(target=car_sensor.get_adc_value,args=(delay_time,),daemon=True)
    t2 = threading.Thread(target=car_interpret.process, args=(brightness_bus,delay_time),daemon=True)
    t3 = threading.Thread(target=car_controll.controll_car, args=(pose_bus,delay_time),daemon=True)
    print('starting threads')
    t1.start()
    t2.start()
    t3.start()
    '''
    
    time.sleep(1)

stop_threads = False
follow_line()
