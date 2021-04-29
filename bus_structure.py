#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 25 13:10:56 2021

@author: nigel
"""


class bus_message():
    def __init__(self):
        self.message =[]
        
    def write(self,data):
        self.message=data
        
    def read(self):
        return self.message