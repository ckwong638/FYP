#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import Sofa.constants.Key as Key
import Sofa.Helper
from Sofa.constants import *
import numpy as np
import math
import csv
import time
import pandas as pd

class FingerController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = a[0]
        self.pressureConstraint = self.node.getChild('finger').cavity.getObject('SurfacePressureConstraint')
        self.dofs = self.node.getChild('finger').getObject('tetras')
        #set up the PID controller
        self.Kp = 0.8
        self.Ki = 0.01
        self.Kd = 0.0
        self.dt = 0.01
        self.flag = True
        self.error = 0.0
        self.intergral_error = 0.0
        self.last_error = 0.0
        self.prev_velocity = np.zeros(3)
        self.desired_angles = [(20, 3.0), (25, 3.0), (20, 3.0)] #each tuple contains the desired angle and its duration
        # self.threshold = 0.1
        self.desired_angle_index = 0
        self.current_duration = 0.0
        self.cumulative_duration = 0.0
        self.last_desired_angle_reached = False
        self.time = 0.0
        
        
        # initialize lists for storing data
        self.time_values = []
        self.angle_values = []
        self.desired_angle_values = []
        self.desired_angle_duration_values = []
        self.control_signal_values = []
        
        # #set the output file path
        self.ouput_path = "finger_controller_data.csv"
        #self.output_path = "/home/siuchi/SOFA/v22.12.00/fyp/Automation_control/details/finger_controller_data.csv"
        
        return

    def calculateAngle(self):
        ##################find out the finger tip position####################
        # node_positions = self.dofs.findData('position').value.tolist()
        # node_velocitys = self.dofs.findData('velocity').value.tolist()
        # min_val = float("inf")
        # max_val = float("-inf")
        # index = -1
        
        # for i in range(len(node_positions)):
        #     if node_positions[i][0] <= min_val and node_positions[i][1] >= max_val and node_positions[i][2] == 0:
        #         min_val = node_positions[i][0]
        #         max_val = node_positions[i][1]
        #         index = i
        ########################################################################
        FingerTipPosition = self.dofs.position.value[30]
        FingerTipVelocity = self.dofs.velocity.value[30]
        current_position = FingerTipPosition
        current_velocity = FingerTipVelocity
        x, y, _ = current_position
        y = y - 30
        current_angle = np.arctan2(y, x)
        current_angle = np.degrees(current_angle)
        current_angle = 180 - current_angle
        # current_angle = int(current_angle)
        # current_angle = round(current_angle, 3)
        
        return current_angle, current_velocity    
    
    def onAnimateBeginEvent(self, event):
        """ called at the beginning of each time step """
        print("AnimatedBegin", event)
        time.sleep(0.01)
        dt_value = event.get('dt')
        self.time += dt_value
        print(self.time)
            
        # calcualte the current angle of the finger
        current_angle, current_velocity = self.calculateAngle()
        
        #check if the last desired angle has been reached
        if self.desired_angle_index >= len(self.desired_angles):
            self.last_desired_angle_reached = True
            return
        
        #get the current desired angle and its duration
        current_desired_angle, current_duration = self.desired_angles[self.desired_angle_index]
        
        #update the current duration for the current desired angle if it's hasn't reached the last desired angle
        if not self.last_desired_angle_reached:
            self.current_duration += dt_value
        
        #check if the current desired angle has been reached for the specified duration
        if self.current_duration >= current_duration:
            # move on to the next desired angle
            self.desired_angle_index += 1
            self.current_duration = 0.0
        
        #calculate the error between the current desired angle and the current angle
        self.error = current_desired_angle - current_angle
        
        #calculate the integral error & derivative error
        self.intergral_error += self.error * self.dt
        derivative_error = (current_velocity - self.prev_velocity) / self.dt
        
        print("The current angle (degree) is: ", current_angle)
        print("The current velocity: ", current_velocity)   

        #compute the PID control signal
        output = self.Kp * self.error + self.Ki * self.intergral_error + self.Kd * derivative_error
        output = np.clip(output, 0, 1.8)
        pressureValue = self.pressureConstraint.value[0]
        self.pressureConstraint.value = output
     
        # #store the value to the data lists
        if self.flag == True:
            self.flag = False
        else:
            self.dt += 0.01
        self.time_values.append(self.time)
        self.angle_values.append(current_angle)
        self.desired_angle_values.append(current_desired_angle)
        self.desired_angle_duration_values.append(self.current_duration)
        self.control_signal_values.append(output[0])
        
        with open('finger_controller_data.csv', mode= 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.time_values)
            writer.writerow(self.angle_values)
            writer.writerow(self.desired_angle_values)
            writer.writerow(self.desired_angle_duration_values)
            writer.writerow(self.control_signal_values)
            
        file.close()
        
        # with open("finger_controller_data.csv") as file:
        #     print (file.read())
                    
        #update the previous velocity and last error for the next time step
        self.prev_velocity = current_velocity
        self.last_error = self.error
        
        print(pressureValue)

    # def onAnimateEndEvent(self, event):
    #     """ called at the end of each time step """
    #     print("AnimatedEnd", event)
    #       # #write data to a CSV file
    #     with open('finger_controller_data.csv', mode= 'w', newline='') as file:
    #         writer = csv.writer(file)
    #         writer.writerow(self.angle_values)
    #         writer.writerow(self.time_values)
    #         writer.writerow(self.control_signal_values[0])
    #         writer.writerow(self.desired_angle_values)
    #     file.close()

    # def onSimulationInitDoneEvent(self, e):
    #     print("Handled event received: " + '8')
   
    def onKeypressedEvent(self, e):
        pressureValue = self.pressureConstraint.value.value[0]
    
        if e["key"] == Key.plus:
            pressureValue += 0.01
            if pressureValue > 1.5:
                pressureValue = 1.5

        if e["key"] == Key.minus:
            pressureValue -= 0.01
            if pressureValue < 0:
                pressureValue = 0
    

        self.pressureConstraint.value = [pressureValue]

        print(pressureValue)
        print("FingerTipPosition is", self.dofs.position.value[30])
        print("FingerTipVelocity is", self.dofs.velocity.value[30])