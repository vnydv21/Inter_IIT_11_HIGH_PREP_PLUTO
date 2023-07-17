import numpy as np

class PID():
    def __init__(self,P,I,D,setpoint,mn,mx):
        self.P = P
        self.I = I
        self.D = D
        self.setpoint = setpoint
        self.arr = [0,0,0,0,0]
        self.mn = mn
        self.mx = mx

    def update(self, val):
        e = val - self.setpoint
        self.arr.append(e)
        self.arr.pop(0)
        return np.clip(int(self.P*e + self.D*(self.arr[4]-self.arr[3])),self.mn,self.mx) # I not required for our purpose, basically PD controller