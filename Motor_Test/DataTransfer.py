# -*- coding: utf-8 -*-
"""
Created on Thu Dec 23 19:38:55 2021

@author: brand
"""

#Libraries
import numpy as np
import matplotlib.pyplot as plt
import serial
import os
from os.path import dirname, join as pjoin
import pandas as pd
import time
import control
from control.matlab import *
filename = "data.txt"

# In[]
arduino_port = "COM6"
baud = 19200
filename = "data.txt"
samples = 1000
print_labels = False
print_labels
TIMEOUT = 2


ser = serial.Serial(arduino_port,baud, timeout = 1)
ser.close()
ser = serial.Serial(arduino_port,baud, timeout = 1)


print("Connected to  Arduino port: " + arduino_port)

file = open(filename, "w")
print("Creating file: " + filename)

try:
    word = (ser.readline()).decode()[:-2]
    start_time = time.time()
    while "Start" not in (word):
        word = (ser.readline()).decode()[:-2]
        load_time = time.time()
        if load_time-start_time > TIMEOUT:
            print('------------------')
            print("Timeout for Data Collection")
            print("Try again")
            print('------------------')
            break
        
    if load_time-start_time < TIMEOUT:
        line = 0
        print('------------------')
        print("Data Collectionx Begins")
        print('------------------')
        while line <= samples:
            data = ((ser.readline()).decode())[:-2]
            print(data)
            
            file = open(filename, "a")
            file.write(data + '\n')
            line = line +1
        
        print('------------------')
        print("Data collection complete!")
        print('------------------')
except:
     print("Error in Reading Data")

ser.close()
file.close()

#Set the save folder to the current path of this Python file
path = os.path.abspath(__file__)  # Python program file
folder_path = pjoin(dirname(path))  # Current directory
os.chdir(folder_path)

# In[]
data = np.loadtxt('openloop.txt', dtype = float, delimiter = ',', skiprows=1)
t = data[:,0]
y = data[:,1]
volt = data[:,2]

plt.figure(figsize = (10,6))
plt.plot(t,y, label = 'Speed')
plt.xlabel('Time [s]', fontsize = 16)
plt.ylabel('Angular Velociyt [rad/s]', fontsize = 16)
plt.xticks(fontsize = 12)
plt.yticks(fontsize = 12)
plt.title('Motor Speed vs Time', fontsize = 20)
plt.legend(fontsize = 12)


plt.figure(figsize = (10,6))
plt.plot(t,volt, label = 'Actual Voltage', lw = 3)
plt.xlabel('Time [s]', fontsize = 16)
plt.ylabel('Angular Velociyt [rad/s]', fontsize = 16)
plt.xticks(fontsize = 12)
plt.yticks(fontsize = 12)
plt.title('Motor Speed vs Time', fontsize = 20)
plt.legend(fontsize = 12)


#Find the max index that the array value is less than or equal than K 
def closestPos(array, K):
    return max(np.nonzero(array <= K)[0])


#Find the slope for the interpolation
def slope(array,i,K):
    return (K - array[i+1])/(array[i+1]-array[i])

#Interpolate to get the result
def interpolate(array, i, m):
    return array[i] + m*(array[i+1] -array[i]);

y_sample = y[600:900]
t_sample = t[600:900]
yss = np.mean(y[700:900])


#y_sample = y[0:250]
#volt_sample = y[0:250]
#t_sample = t[0:250]
#yss = np.mean(y_sample[25:250])

volt_ss = 10
K = yss/volt_ss
print(K)

y_tau = (1-np.exp(-1))*yss
pt_tau1 = closestPos(y_sample,y_tau) #Index of the Pr2 in the Pr array
m = slope(y_sample,pt_tau1,y_tau)
tau_stamp = interpolate(t_sample, pt_tau1, m)

pt_tau2 = closestPos(y_sample,0.001) #Index of the Pr2 in the Pr array
tau = tau_stamp - t_sample[pt_tau2]

print(tau)

RPM_TO_RADS = 0.104719755
emax = 270*RPM_TO_RADS #(rads/s)

# In[]
data = np.loadtxt(filename, dtype = float, delimiter = ',', skiprows=1)
t = data[:,0]
r = data[:,1]*0.104719755 #[rad/s]
y = data[:,2]*0.104719755 #[rad/s]
e = data[:,3]*0.104719755 #[rad/s]
volt = data[:,4]*volt_ss/12


kp = 1
ki = 15
kd = 0.00

w0 = 25*2*np.pi

s = tf('s')
P = K/(tau*s+1)
C = kp + ki/s + kd*s
F = w0/(s+w0)
L = P*C
Gyr = feedback(L, 1)
Gyu = C*feedback(1,P*C)

tt = np.linspace(0,.87,1000)
yt1,tt = step(Gyr,tt)
ut1,tt = step(Gyu,tt)
yt1 = emax*yt1
ut1 = emax*ut1

yt2 = yt1[-1]-yt1
ut2 = ut1[-1]-ut1


tt = np.linspace(0,1.74,2000)
yt = np.concatenate((yt1,yt2), axis = 0)
ut = np.concatenate((ut1,ut2), axis = 0)


plt.figure(figsize = (12,6))
plt.plot(t,r, label = 'Referene Speed', linestyle = '--', color = 'k', lw = 3)
plt.plot(t,y, label = 'Actual Speed')
plt.plot(t,e, label = 'Error')
plt.plot(tt,yt, label = 'Theorectical Speed')
plt.xlabel('Time [s]', fontsize = 16)
plt.ylabel('Angular Velociyt [rpm]', fontsize = 16)
plt.xticks(fontsize = 12)
plt.yticks(fontsize = 12)
plt.title('Motor Speed vs Time', fontsize = 20)
plt.legend(fontsize = 12)


plt.figure(figsize = (12,6))
plt.plot(t,volt, label = 'Actual Voltage', lw = 3)
plt.plot(tt,ut, label = 'Theorectical Voltage', lw = 3)
plt.xlabel('Time [s]', fontsize = 16)
plt.ylabel('Angular Velociyt [rad/s]', fontsize = 16)
plt.xticks(fontsize = 12)
plt.yticks(fontsize = 12)
plt.title('Motor Speed vs Time', fontsize = 20)
plt.legend(fontsize = 12)


# In[]
from scipy import integrate

data = np.loadtxt(filename, dtype = float, delimiter = ',', skiprows=1)
t = data[:,0]
r = data[:,1]*0.104719755 #[rad/s]
y = data[:,2]*0.104719755 #[rad/s]
e = data[:,3]*0.104719755 #[rad/s]
volt = data[:,4]*volt_ss/12

def mysat(vin):
    if vin > volt_ss:
        vout = volt_ss
    elif vin < -volt_ss:
        vout = -volt_ss
    else:
        vout = vin
    return vout

temp = []

def Euler(x0,t,kp,ki,kd,theta_d,dt,use_sat=True):
    x = np.zeros(len(t))
    xdot = np.zeros(len(t))
    U = np.zeros(len(t))
    x[0] = x0
    cum_Error = 0
    past_Error = 0
    for ii in range(len(t)):    
        theta = x[ii]
        Error = theta_d - theta
        cum_Error = cum_Error + (Error + past_Error) / 2 * dt
        past_Error = Error
        u = kp*Error + ki*cum_Error
        
        if use_sat:
            u = mysat(u)
        theta_dot = -1/tau*theta + K/tau*u
        
        
        if ii < len(t)-1:
            x[ii+1] = x[ii] + theta_dot*dt
        xdot[ii] = theta_dot
        U[ii] = u

    return x,xdot,U
dt = t[1]-t[0]

ind_t1 = (np.where(r == 0))[0][0]
t1 = t[0:ind_t1]
ind_t2 = (np.where(r[ind_t1:] == r[0]))[0][0]+ind_t1
t2 = t[ind_t1:ind_t2]

y_theo1,_,u_theo1 = Euler(0,t1,kp,ki,kd,r[0],dt,True)
y_theo2,_,u_theo2 = Euler(y_theo1[-1],t2,kp,ki,kd,0,dt,True)
tt = np.concatenate((t1,t2), axis = 0)
y_theo = np.concatenate((y_theo1,y_theo2), axis = 0)
u_theo = np.concatenate((u_theo1,u_theo2), axis = 0)


# In[]



plt.figure(figsize = (12,6))
plt.plot(t,r, label = 'Referene Speed', linestyle = '--', color = 'k', lw = 3)
plt.plot(t,y, label = 'Actual Speed')
plt.plot(t,e, label = 'Error')
plt.plot(tt,y_theo, label = 'Saturated')
plt.xlabel('Time [s]', fontsize = 16)
plt.ylabel('Angular Velociyt [rpm]', fontsize = 16)
plt.xticks(fontsize = 12)
plt.yticks(fontsize = 12)
plt.title('Motor Speed vs Time', fontsize = 20)
plt.legend(fontsize = 12)


plt.figure(figsize = (12,6))
plt.plot(t,volt, label = 'Actual Voltage', lw = 3)
plt.plot(tt,u_theo, label = 'Theorectical Voltage', lw = 3)
plt.xlabel('Time [s]', fontsize = 16)
plt.ylabel('Angular Velociyt [rad/s]', fontsize = 16)
plt.xticks(fontsize = 12)
plt.yticks(fontsize = 12)
plt.title('Motor Speed vs Time', fontsize = 20)
plt.legend(fontsize = 12)



