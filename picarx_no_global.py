#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 18:03:14 2021

@author: nigel
"""


import  time
try:
    from  ezblock  import *
    from ezblock import __reset_mcu__
    __reset_mcu__()
    time.sleep (0.01)
except  ImportError:
    print ("This  computer  does  not  appear  to be a PiCar -X system(/opt/ezblock  is not  present). Shadowing  hardware  calls with  substitute  functions ")
    from  sim_ezblock  import *
import time
import atexit
import math
import logging
#from logdecorator import log_on_start, log_on_end, log_on_error
logging_format = '%(asctime)s: %(message)s'
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt='%H:%M:%S')
logging.getLogger().setLevel(logging.DEBUG)
'''
PERIOD = 4095
PRESCALER = 10
TIMEOUT = 0.02

dir_servo_pin = Servo(PWM('P2'))
camera_servo_pin1 = Servo(PWM('P0'))
camera_servo_pin2 = Servo(PWM('P1'))
left_rear_pwm_pin = PWM("P13")
right_rear_pwm_pin = PWM("P12")
left_rear_dir_pin = Pin("D4")
right_rear_dir_pin = Pin("D5")

S0 = ADC('A0')
S1 = ADC('A1')
S2 = ADC('A2')

Servo_dir_flag = 1
dir_cal_value = 0
cam_cal_value_1 = 0
cam_cal_value_2 = 0
motor_direction_pins = [left_rear_dir_pin, right_rear_dir_pin]
motor_speed_pins = [left_rear_pwm_pin, right_rear_pwm_pin]
cali_dir_value = [1, -1]
cali_speed_value = [0, 0]
'''

class picar_thing(): # i was pretty sure they weren't going to use 'thing' in their modules so it wouldn't clash with any of their code
    def __init__(self):
        self.PERIOD = 4095
        self.PRESCALER = 10
        self.TIMEOUT = 0.02
        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')
        self.Servo_dir_flag = 1
        self.dir_cal_value = 0
        self.cam_cal_value_1 = 0
        self.cam_cal_value_2 = 0
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)
        atexit.register(self.cleanup)
    
    def cleanup(self): #sets the wheels forwards and stops the motors
        self.set_motor_speed(1,0)
        self.set_motor_speed(2,0)
        self.set_dir_servo_angle(0)
    
    def set_motor_speed(self, motor, speed): # sets the motor speed
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
    
    def motor_speed_calibration(self, value): # finds the motor calibration speed value
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0
    
    def motor_direction_calibration(self, motor, value): # calibrates the diretion of the motor
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1*self.cali_dir_value[motor]
    
    
    def dir_servo_angle_calibration(self, value): # calibrates the servo angle
        self.dir_cal_value = value
        self.set_dir_servo_angle(self.dir_cal_value)
        # dir_servo_pin.angle(dir_cal_value)
    
    def set_dir_servo_angle(self, value): # sets the wheel orientation. +33 is there to compensate for the servo/axle alignment
        self.dir_servo_pin.angle(value+self.dir_cal_value+33)
    
    def camera_servo1_angle_calibration(self, value):  # calibrates one of the camera servos
        self.cam_cal_value_1 = value
        self.set_camera_servo1_angle(self.cam_cal_value_1)
        # camera_servo_pin1.angle(cam_cal_value)
    
    def camera_servo2_angle_calibration(self, value): # calibrates the other camera servo
        self.cam_cal_value_2 = value
        self.set_camera_servo2_angle(self.cam_cal_value_2)
        # camera_servo_pin2.angle(cam_cal_value)
    
    def set_camera_servo1_angle(self, value): # sets one of the camera servos
        self.camera_servo_pin1.angle(-1 *(value+self.cam_cal_value_1))
    
    def set_camera_servo2_angle(self, value): # sets the other camera servo
        self.camera_servo_pin2.angle(-1 * (value+self.cam_cal_value_2))
    
    def get_adc_value(self): # finds the values going to the ADC pins
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list
    
    def set_power(self, speed): # sets a speed to the motors
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 
    
    def adjust_speed(self, speed,angle): # tweaks the speed of the motors to turn more smoothly
        r=math.tan(angle/180*math.pi)/4+2.3
        return [speed*r/(r-2.3),speed*r/(r+2.3)]
    
    def backward(self, speed=40,angle=0): # moves the car backwards at a given speed and angle
        speeds=self.adjust_speed(speed,angle)
        self.set_motor_speed(1, speeds[0])
        self.set_motor_speed(2, speeds[1])
    
    def forward(self, speed=40,angle=0): # moves the car forwards at a given speed and angle
        speeds=self.adjust_speed(speed,angle)
        self.set_motor_speed(1, -1*speeds[0])
        self.set_motor_speed(2, -1*speeds[1])
    
    def stop(self): # kills the motors
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
    
    def Get_distance(self): # who knows
        timeout=0.01
        trig = Pin('D8')
        echo = Pin('D9')
    
        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm
    
    def kturn(self, flag): # performs a k-turn
        if flag=='left':
            mult=-1
        else:
            mult=1
        self.set_dir_servo_angle(40*mult)
        self.forward()
        time.sleep(1.2)
        self.stop()
        self.backward()
        time.sleep(1.8)
        self.stop()
        self.set_dir_servo_angle(40*mult)
        self.forward()
        time.sleep(1.2)
        self.stop()
    
    def parallel(self): #performs a parallel park
        self.stop()
        self.backward()
        time.sleep(0.5)
        self.set_dir_servo_angle(60)
        time.sleep(1)
        self.set_dir_servo_angle(-60)
        time.sleep(1)
        self.stop()
    
    def test(self): #checks functionality
        # dir_servo_angle_calibration(-10) 
        #kturn('right')
        #time.sleep(1)
        #kturn('left')
        #get_maneuver()
        self.forward(40,20)
        time.sleep(2)
        self.backward(40,-20)
        time.sleep(2)
        self.stop()
    
    def get_maneuver(self): # requests an input from the user and runs the desired command
        flag=False
        while not(flag):
            flag=True
            a=input('choose maneuver: kl=k-turn left, kr=k-turn right, p=parallel park, f=forwards, b=backwards ##=set angle at ## degrees, n=no maneuver')
            if a == 'kl':
                self.kturn('left')
            elif a=='kr':
                self.kturn('right')
            elif a== 'p':
                self.parallel()
            elif a=='f':
                self.forward()
                time.sleep(1)
            elif a=='b':
                self.backward()
                time.sleep(1)
            elif a.isdecimal():
                b=float(a)
                self.set_dir_servo_angle(b)
                time.sleep(0.5)
            elif a=='n':
                pass
            else:
                print('not valid maneuver')
                flag=False
        self.stop()

if __name__ == "__main__":
    new_car=picar_thing()
    try:
        # dir_servo_angle_calibration(-10) 
        new_car.test()
    finally: 
        new_car.stop()
