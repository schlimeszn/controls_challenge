from . import BaseController
import numpy as np

class Controller(BaseController):
  """
  A simple PID controller
  """
  def __init__(self, kp=0.045, ki=0.09501335, kd=-0.03649474):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.prev_error = 0
    self.integral = 0
    
  def update(self, target_lataccel, current_lataccel, state, future_plan):
    error = target_lataccel - current_lataccel
    self.integral += error
    derivative = error - self.prev_error
    
    output = self.kp * error + self.ki * self.integral + self.kd * derivative
    self.prev_error = error
    return output 
