# V4
# Based on the included documentation for RTIMULib2 and PCA9685
# This code:
# 1. Reads IMU measurements and performs sensor fusion using RTIMULib2
# 2. Reads whether the power is on using a program implemented on the Atmega328p [Custom code]
# 3. Controls motors using PWM. Also sets up the motor limits and releases the brakes
# 4. PID is implemented but it's not possible to tune it due to the sensor latency (plant is faster than sensor)
# 5. Writes logs to desktop with flight data for sys ident
# Thomas Pile, Sheffield Hallam Uni, August 2018

from __future__ import division
import time
from multiprocessing.dummy import Pool as ThreadPool

# Import the PCA9685 module.
import Adafruit_PCA9685

import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math

import numpy as np # matrix mathematics package
import spidev # SPI


# Global Variables
rpyref = np.array([0,0,0])
vxyz = np.array([0,0,0])
xyz = np.array([0,0,0])
headingT = 0
speedKM = 0
out1 = 0
out2 = 0
out3 = 0
out4 = 0
laser1 = 0
laser2 = 0
fcsenabled = 0
pwmupdatedelaycounter = 0
pwmupdatedelay = 100 # have to do this or the AVR goes wrong!

# ---------------------------------------------------------------
# IMU Setup
# ---------------------------------------------------------------

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters
imu.setSlerpPower(0.01) # measured state influence. 0 to 1. 0.02 orig
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

# ---------------------------------------------------------------
# AVR Setup
# ---------------------------------------------------------------
# Setup AVR servo interface
# 3.9mhz  = 3900000
# 1953mhz = 1953000
# 488khz  = 488000
# 244khz  = 244000
# 122khz  = 122000
bus = 0x00
device = 0x00 # 0x00 for the small board, 0x01 for the cnc board
spi = spidev.SpiDev()
spi.open(bus,device)
spi.max_speed_hz = 122000
spi.mode = 0b00
print("SPI Setup bus:0 dev:0")

# open log file
log = open('logfile.csv','w')
log.write('$, Roll, Pitch, Yaw, Velocity X, Velocity Y, Velocity Z, heading T, speedKM, rc_ch1_roll, rc_ch2_thrust, rc_ch3_pitch, fcs_enabled, ultrasonic_height, out1, out2, out3, out4, laser1, laser2, dt(nanoseconds)\r\n')
    
# ---------------------------------------------------------------
# PCA9686 Servo Setup
# ---------------------------------------------------------------
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)


# File reading
#file1 = open("file.txt")
#file1.read();
#file1.readline()
#for eachline in file1: #step through
#  print(file1.line())


# servo values
servo1 = 1
servo2 = 1
servo3 = 1
servo4 = 1
servo5 = 1
motorsetupcomplete = 0;


rpyeprev = 0

while True:

  if imu.IMURead():
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    # transfer the rpy measurements to a better array and convert to degrees
    rpy = np.array([math.degrees(fusionPose[0]),math.degrees(fusionPose[1]),math.degrees(fusionPose[2])]) 
  
  rpye = np.subtract(rpyref,rpy)
  print("r_err: %f p_err: %f y_err: %f" % (rpye[0],rpye[1],rpye[2]))
  
  # P controller
  dt = 0.04
  kp = 10 # 0.3
  ki = 0
  kd = 30
  # P
  up = np.multiply(rpye,kp)
  # D
  ederiv = (rpye-rpyeprev)
  ud = np.multiply(np.divide(ederiv,dt),kd)
  rpyeprev = rpye
  # I
  ui = np.multiply(np.multiply(rpye,ki),dt)
  u = np.add(up,ui,ud)

  # control distribution
  # roll
  if u[0]>0:
    out1 = round(abs(u[0]))
    #out2 = -round(abs(u[0]))
  if u[0]<0:
    #out1 = -round(abs(u[0]))
    out2 = round(abs(u[0]))
  
  # check whether power is on
  packet = [0x31, 0x00]  
  servopoweron = spi.xfer(packet)
  #print(servopoweron[0])
  
  # 0->1: motor not set up yet, power detected so doing a setup  
  if(motorsetupcomplete<1):
      # if power is on
      if(servopoweron[0]==1):
          # motor set up for the PCA
          pwm.set_pwm(0, 0, 200)
          pwm.set_pwm(1, 0, 200)
          pwm.set_pwm(2, 0, 200)
          pwm.set_pwm(3, 0, 200)
          time.sleep(2)
          pwm.set_pwm(0, 0, 600)
          pwm.set_pwm(1, 0, 600)
          pwm.set_pwm(2, 0, 600)
          pwm.set_pwm(3, 0, 600)
          time.sleep(2)
          pwm.set_pwm(0, 0, 200)
          pwm.set_pwm(1, 0, 200)
          pwm.set_pwm(2, 0, 200)
          pwm.set_pwm(3, 0, 200)
          time.sleep(2)
          
          motorsetupcomplete=1 # motor test mode
          motorsetupcomplete=1 # normal operation
          print("Motor setup Complete")
      # end        
  # end

  # 1: setup completed
  if(motorsetupcomplete==1):
      #
      time.sleep(2)
      pwm.set_pwm(0, 0, 250)
      pwm.set_pwm(1, 0, 250)
      pwm.set_pwm(2, 0, 250)
      pwm.set_pwm(3, 0, 250)
      time.sleep(2)
      pwm.set_pwm(0, 0, 200)
      pwm.set_pwm(1, 0, 200)
      pwm.set_pwm(2, 0, 200)
      pwm.set_pwm(3, 0, 200)

      motorsetupcomplete=2
      
  # end

  # read rc input 1
  # 0x11, 0x12, 0x13, 0x14
  FROM8BitSERVO = 1
  packet = [0x11, 0x00]  
  rcin1 = spi.xfer(packet)
  # subtract base value. For ch1-4 these are 0, 70, 28, 50. top vals are 0, 250, 248, 228
  rcin1 = rcin1[0] - 0
  rcin1 = rcin1 * FROM8BitSERVO
  #print(rcin1[0])

  # read rc input 2 - roll
  # -160 so zero is with stick at the centre
  packet = [0x12, 0x00]
  rcin2 = spi.xfer(packet)
  #print(rcin2[0])
  rcin2 = rcin2[0] - 160
  rcin2 = rcin2 * FROM8BitSERVO
    
  # read rc input 3 - thrust
  # minus 26 so thrust starts from zero, up to 248 ish
  packet = [0x13, 0x00]
  rcin3 = spi.xfer(packet)
  rcin3 = rcin3[0] - 26
  rcin3 = rcin3 * FROM8BitSERVO
  #print(rcin3[0])
    
  # read rc input 4 - pitch
  # centre: 154 to 158 ish
  packet = [0x14, 0x00]
  rcin4 = spi.xfer(packet)
  #print(rcin4[0])
  rcin4 = rcin4[0] - 154
  rcin4 = rcin4 * FROM8BitSERVO
  #print(rcin4[0])

  # rc vales in a vector: thrust, roll, pitch, yaw. yaw isn't working
  rcin = [rcin3, rcin2, rcin4, rcin1]
    
  # autopilot enabled check function
  packet = [0x30, 0x00]
  spi.xfer(packet)
  packet = [0x30, 0x00]
  fcsenabledtmp = (spi.xfer(packet))
  fcsenabled = fcsenabledtmp[0]
  #print(fcsenabled)

  # ultrasonic ranging
  packet = [0x31, 0x00]
  spi.xfer(packet)
  packet = [0x31, 0x00]
  sonicheight = spi.xfer(packet)
  #print(sonicheight[0])
  
  # actual output to servos
  # outx should range 0 to max, then the base value is added to it here
  if(motorsetupcomplete==2):
        #
        pwmbase = 250
        servo1 = int(pwmbase+out1)
        servo2 = int(pwmbase+out2)
        servo3 = int(pwmbase+out3)
        servo4 = int(pwmbase+out4)
        
        # lower limit
        if (servo1<210): servo1=210
        if (servo2<210): servo2=210
        if (servo3<210): servo3=210
        if (servo4<210): servo4=210
        # upper limit
        if (servo1>600): servo1=600
        if (servo2>600): servo2=600
        if (servo3>600): servo3=600
        if (servo4>600): servo4=600
        
        pwm.set_pwm(0, 0, servo1)
        pwm.set_pwm(1, 0, servo2)
        pwm.set_pwm(2, 0, servo3)
        pwm.set_pwm(3, 0, servo4)
        #print('motor 1 out: ', servo1)
        #print('motor 2 out: ', servo2)

        # if power goes low then go back to motor setup
        if(servopoweron[0]==0):
            motorsetupcomplete=2 
  # end
    
  # Logging, also need this
  log.write('0, ')
  log.write('{}'.format(rpy[0])) # roll
  log.write(', ')
  log.write('{}'.format(rpy[1])) # pitch
  log.write(', ')
  log.write('{}'.format(rpy[2])) # yaw
  log.write(', ')
  log.write('{}'.format(vxyz[0])) # velocity X
  log.write(', ')
  log.write('{}'.format(vxyz[1])) # velocity Y
  log.write(', ')
  log.write('{}'.format(vxyz[2])) # velocity Z
  log.write(', ')
  log.write('{}'.format(headingT)) # heading T
  log.write(', ')
  log.write('{}'.format(speedKM)) # speed KM
  log.write(', ')
  log.write('{}'.format(rcin[1])) # rc_ch1_roll
  log.write(', ')
  log.write('{}'.format(rcin[0])) # rc_ch2_thrust
  log.write(', ')
  log.write('{}'.format(rcin[2])) # rc_ch3_pitch
  log.write(', ')
  log.write('{}'.format(fcsenabled)) # fcs enabled
  log.write(', ')
  log.write('{}'.format(sonicheight[0])) # ultrasonic height
  log.write(', ')
  log.write('{}'.format(servo1)) # out 1
  log.write(', ')
  log.write('{}'.format(servo2)) # out 2
  log.write(', ')
  log.write('{}'.format(servo3)) # out 3
  log.write(', ')
  log.write('{}'.format(servo4)) # out 4
  log.write(', ')
  log.write('{}'.format(laser1)) # laser 1
  log.write(', ')
  log.write('{}'.format(laser2)) # laser 2
  log.write(', ')
  log.write('{}'.format(dt)) # dt 
  log.write(' \n')
    
  # sleep
  dt = poll_interval*1.0/1000.0 # 4ms
  #print('{}'.format(dt))
  #time.sleep(dt)

    
#log.close()


