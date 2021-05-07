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
import threading
import concurrent.futures
#from bus_structure import bus_message
from rossros import *

class sensor():

    def __init__(self,delay_time=0.5):
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')
        self.delay_time=delay_time
        
    def get_adc_value(self): # finds the values going to the ADC pins
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list
        #self.brightness_bus.write(adc_value_list)
        #time.sleep(self.delay_time)
    
class interpreter():

    def __init__(self, sensitivity = 100, polarity = 1, delay_time=0.5):

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
        
        self.delay_time=delay_time
        
    def process(self,sensor_value):
        pos = 0

        if (self.polarity*(sensor_value[1] - sensor_value[0])) > self.sensitivity:
            pos = -0.5
        elif (self.polarity*(sensor_value[1] - sensor_value[2])) > self.sensitivity:
            pos = 0.5
        elif (self.polarity*(sensor_value[1] - sensor_value[0])) > self.sensitivity:
            pos = -1
        elif (self.polarity*(sensor_value[1] - sensor_value[2])) > self.sensitivity:
            pos = 1
        return pos
            #self.interpreter_bus.write(pos)
            #time.sleep(self.delay_time)
        
class controller():

    def __init__(self, scaling_factor=40, delay_time=0.5):
        
        self.scaling_factor = scaling_factor
        self.car = picar_thing()
        self.delay_time = delay_time
    def controll_car(self, pos, usonic=100):
        
        if usonic <= 10:
            self.car.stop()
        else:
            self.car.set_dir_servo_angle(pos*self.scaling_factor)
            self.car.forward(15, pos*self.scaling_factor)
        
        time.sleep(self.delay_time)


class ultrasonic():
    
    def __init__(self):
        self.trig = Pin('D8')
        self.echo = Pin('D9')
        
    def Get_distance(self): # who knows
        timeout=0.01
    
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm
    
def follow_line(*args):
    global stop_threads
    if len(args) == 3:   
        car_sensor = sensor()
        car_interpret = interpreter(args[0], args[1])
        car_controll = controller(args[2])
        car_ultrasonic = ultrasonic()
    else:
        car_sensor = sensor()
        car_interpret = interpreter()
        car_controll = controller()
        car_ultrasonic = ultrasonic()
    #print(car_sensor.get_adc_value)
    adc_bus=Bus([0,0,0],name='adc_bus')
    control_bus=Bus(0,name='control_bus')
    distance_bus=Bus(0,name='ultrasonic_bus')
    sensing=Producer(car_sensor.get_adc_value,adc_bus,name='sensor',delay=0.1)
    interpreting=ConsumerProducer(car_interpret.process,input_busses=adc_bus,output_busses=control_bus,name='interpreter',delay=0.1)
    controlling=Consumer(car_controll.controll_car,input_busses=(control_bus,distance_bus),name='controller',delay=0.1)
    distance=Producer(car_ultrasonic.Get_distance,distance_bus,name='ultrasonic',delay=0.1)
    '''
    with concurrent.futures.ThreadPoolExecutor(max_workers = 4) as executor:
        eSensor = executor.submit(sensing)
        eInterpreter = executor.submit(interpreting)
        eController = executor.submit(controlling)
        eStop.result()
        eSensor.result()
        eInterpreter.result()
        eController.result()
    ''' 
    print('making threads')
    t1 = threading.Thread(target=sensing, args=(),daemon=True)
    time.sleep(1)
    t2 = threading.Thread(target=interpreting, args=(),daemon=True)
    time.sleep(1)
    t3 = threading.Thread(target=controlling, args=(),daemon=True)
    print('starting threads')
    t1.start()
    t2.start()
    t3.start()
    
    
    time.sleep(20)

stop_threads = False
follow_line()
