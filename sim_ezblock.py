#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 15:25:29 2021

@author: nigel
"""


import time

class _Basic_class(object):
    def __init__(self):
        pass

    @property
    def debug(self):
        return 'nope'

    @debug.setter
    def debug(self, debug):
        pass
    
    def run_command(self, cmd):
        return 0, 0

    def map(self, x, in_min, in_max, out_min, out_max):
        return 0

class Servo(_Basic_class):
    MAX_PW = 2500
    MIN_PW = 500
    _freq = 50
    def __init__(self, pwm):
        pass

    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        pass

def test():
    pass
    


class Pin(_Basic_class):

    def __init__(self, *value):
        pass
        
    def check_board_type(self):
        pass

    def init(self, mode, pull):
        pass

    def dict(self, *_dict):
        pass

    def __call__(self, value):
        return 1

    def value(self, *value):
        return 1

    def on(self):
        return 1

    def off(self):
        return 1

    def high(self):
        return 1

    def low(self):
        return 1

    def mode(self, *value):
        pass

    def pull(self, *value):
        return 1

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        return "GPIO%s"%1

    def names(self):
        return ['this', 'dumb']

    class cpu(object):
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4  = 4
        GPIO5  = 5
        GPIO6  = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass

class I2C(_Basic_class):
    def __init__(self, *args, **kargs):  
        pass

    def _i2c_write_byte(self, addr, data):
        return 1
    
    def _i2c_write_byte_data(self, addr, reg, data):
        return 1
    
    def _i2c_write_word_data(self, addr, reg, data):
        return 1
    
    def _i2c_write_i2c_block_data(self, addr, reg, data):
        return 1
    
    def _i2c_read_byte(self, addr): 
        return 1

    def _i2c_read_i2c_block_data(self, addr, reg, num):
        return 1

    def is_ready(self, addr):
        return True

    def scan(self):
        return 'path'

    def send(self, send, addr, timeout=0): 
        pass

    def recv(self, recv, addr=0x00, timeout=0):
        return 1

    def mem_write(self, data, addr, memaddr, timeout=5000, addr_size=8): #memaddr match to chn
        pass
    
    def mem_read(self, data, addr, memaddr, timeout=5000, addr_size=8):
        return 1
    
    def readfrom_mem_into(self, addr, memaddr, buf):
        return 1
    
    def writeto_mem(self, addr, memaddr, data):
        pass
    
class PWM(I2C):
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44

    ADDR = 0x14

    CLOCK = 72000000

    def __init__(self, channel, debug="critical"):
        pass

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        pass

    def prescaler(self, *prescaler):
        pass

    def period(self, *arr):
        pass

    def pulse_width(self, *pulse_width):
        pass

    def pulse_width_percent(self, *pulse_width_percent):
        pass

class ADC(I2C):
    def __init__(self, chn):
        pass
        
    def read(self):       
        return 1

    def read_voltage(self):
        return 1
        

if __name__ == "__main__":
    test()
    
    