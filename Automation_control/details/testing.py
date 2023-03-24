import Sofa
import Sofa.constants.Key as Key
import Sofa.Helper
from Sofa.constants import *
import numpy as np
import math
import csv
import time

class FingerController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = a[0]
        self.pressureConstraint = self.node.getChild('finger').cavity.getObject('SurfacePressureConstraint')
        self.dofs = self.node.getChild('finger').getObject('tetras')
        #set up the PID controller
        self.Kp = 0.3
        self.Ki = 0
        self.Kd = 0.0
        self.dt = 0.01
        self.error = 0.0
        self.intergral_error = 0.0
        self.last_error = 0.0
        self.prev_velocity = np.zeros(3)
        self.desired_angle = 20
        
        return
    
    def calculateAngle(self):
        FingerTipPosition = self.dofs.position.value[30]
        FingerTipVelocity = self.dofs.velocity.value[30]
        current_position = FingerTipPosition
        current_velocity = FingerTipVelocity
        x, y, _ = current_position
        y = y - 30
        current_angle = np.arctan2(y, x)
        current_angle = np.degrees(current_angle)
        current_angle = 180 - current_angle
        
        return current_angle, current_velocity    
    
    def onAnimateBeginEvent(self, event):
        """ called at the beginning of each time step """
        print("AnimatedBegin", event)
        time.sleep(0.01)
        dt_value = event.get('dt')
        # calcualte the current angle of the finger
        current_angle, current_velocity = self.calculateAngle()
        
        #calculate the error between the desired angle and the current angle
        self.error = self.desired_angle - current_angle
        
        #calculate the integral error & derivative error
        self.intergral_error += self.error * self.dt
        derivative_error = (current_velocity - self.prev_velocity) / self.dt
        
        print("The current angle(degree) is: ", int(current_angle))
        print("The current velocity: ", current_velocity)
        print(self.intergral_error)
        
        #compute the PID control signal
        output = self.Kp * self.error + self.Ki * self.intergral_error + self.Kd * derivative_error
        pressureValue = self.pressureConstraint.value[0]
        self.pressureConstraint.value = output
        
        # set the upper limit and lower limit of the pressure value
        if pressureValue > 1.5:
                pressureValue = 1.5
                
        if pressureValue < 0:
                pressureValue = 0
                
        #update the previous velocity and last error for the next time step
        self.prev_velocity = current_velocity
        self.last_error = self.error
        
        print(pressureValue)