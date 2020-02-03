#!/usr/bin/env python3

"""

gaggia_tisc_v1.py: This is a simple python script to run a PID temp controller on a
Gaggia Classic coffee machine using a raspberry pi and a TSIC306 temp sensor.

__author__      = "Travis Dobias"
__copyright__   = "Copyright 2020"
__email__       = "travis@dobias.be"

For temperature monitoring, this uses a TSIC style probe attached to the
outside of the boiler.  NB this doesn't support the steam feature yet.

This code uses the tsic.py code from:
https://github.com/grillbaer/python-tsic

This code also uses the PIGPIO library.

For boiler control, this uses a FOSTEK style SSR rated 25A
For Pump control, this uses a FOSTEK style SSR rated 25A

Please make sure you are familiar with household electricity!  Disconnect
the coffee machine from the main current before working inside the machine.

# GPIO PINs

# PIN 1  - 3V3 for TSIC306
# PIN 9  - ground for TSIC306
# PIN 11  - GPIO17 for TSIC306

# PIN 14 - ground for Fostek boiler
# PIN 16 - GPIO23 output to control boiler SSR
# PIN 18 - GPIO24 output to control pump SSR - yellow wire
# PIN 20 - ground - orange wire for SSR
# PIN 17 - 3.3v for brew switch - brown wire
# PIN 13 - GPIO 27 for brew switch - green wire

# PIN 4 - 5v for LCD
# PIN 6 - ground for LCD
# PIN 3 - SCA for LCD
# PIN 5 - SCL for LCD

# PIN 40 - GPIO21 contact switch for steam on/off
# PIN 39 - ground for pin 22

# PIN 36 - GPIO16 contact switch for increment setpoint
# PIN 34 - ground for pin 36

# PIN 32 - GPIO 12 contact switch for flush
# PIN 30 - Ground for pin 32

you can run the program from a ssh session, but if the session hangs, the
program will hang, and the boiler should cut off.

Another way of running it is with the no-hang up feature:
sudo nohup python3 gaggia_tsic_v1.py &

or add a startup script as described here:
https://www.instructables.com/id/Raspberry-Pi-Launch-Python-script-on-startup/

"""

import os
import glob
import pigpio
from tsic import TsicInputChannel, Measurement, TSIC306
import time
import RPi.GPIO as GPIO
import signal
import sys
import I2C_LCD_driver
import json
from datetime import datetime

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.output(24,GPIO.LOW) # switch off pump in case it is already on
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #brew
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP) #steam
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP) #change temp
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP) #change temp

mylcd = I2C_LCD_driver.lcd()
pi = pigpio.pi()
tsic = TsicInputChannel(pigpio_pi=pi, gpio=17, tsic_type=TSIC306)

class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self,signum, frame):
    self.kill_now = True

# below is the PID controller, could be written as a function, but I created
# it as a class because why not.

class PIDController:
    def __init__(self, setpoint, antiwindup, Kp, Kd, Ki, name):
        self.setpoint = setpoint
        self.previous_delta = 0
        self.delta = 0
        self.antiwindup = antiwindup
        self.integral = 0
        self.derivative = 0
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.dt = 1
        self.output = 0
        self.boiler = 0
        self.sensor_reading = 0
        self.name = name

    def calc(self, x):

        # Basic PID forumla with an antiwindup feature
        self.sensor_reading = x
        self.previous_delta = self.delta
        self.delta = self.setpoint - self.sensor_reading
        if self.sensor_reading > self.setpoint-self.antiwindup:
            if self.sensor_reading < self.setpoint+self.antiwindup:
                self.integral = self.integral + (self.delta * self.dt)
        self.derivative = (self.delta - self.previous_delta) / self.dt
        self.output = int((self.delta * self.Kp) + (self.Kd * self.derivative) + (self.integral * self.Ki))

        # heat the boiler elements if the PID output is positive

        if self.output>0:
            if self.sensor_reading>0 and self.sensor_reading<self.setpoint: #safety catch incase PID is in error state
                if self.output>100:
                    self.output=100
                boilerPWM.start(self.output)
                self.boiler=self.output
            else:
                boilerPWM.stop()
        else:
            boilerPWM.stop()
            self.boiler=0

        # debug features
        if settings["verbose"]:
            print(time.strftime("%H:%M:%S"), "{0:8.3f}".format(self.sensor_reading), "{0:8.0f}".format(self.boiler), "{0:8.1f}".format(self.output),"{0:8.1f}".format(self.delta * self.Kp),"{0:8.1f}".format(self.integral * self.Ki),"{0:8.1f}".format(self.Kd * self.derivative),"{0:>8s}".format(self.name))
        time.sleep(.3)

# def logging():
#     if settings["verbose"]:
#         print(time.strftime("%H:%M:%S"), "{0:8.3f}".format(self.sensor_reading), "{0:8.0f}".format(self.boiler), "{0:8.1f}".format(self.output),"{0:8.1f}".format(self.delta * self.Kp),"{0:8.1f}".format(self.integral * self.Ki),"{0:8.1f}".format(self.Kd * self.derivative),"{0:>8s}".format(self.name))


def read_temp():

        measurement = tsic.measure_once(timeout=1.0)
        temp_c = measurement.degree_celsius
        return temp_c

# this function is triggered when the brewbutton is switched on
def Brew():

    #preheat boiler elements for 1 second before pumping
    mylcd.lcd_display_string("{:5d}".format(100)+"% PRE",1,6)
    boilerPWM.start(100)
    time.sleep(1)
    boilerPWM.stop()

    # start pump (gaggia pump switch is rewired to a FOSTEK SSR)
    GPIO.output(24,GPIO.HIGH)

    # this runs the boiler while the pump is brewing to maintain the water
    # temp as the boiler pulls in cold water from the tank

    while GPIO.input(27):

        sensor_reading=read_temp()
        BRW.calc(sensor_reading)
        mylcd.lcd_display_string("{0:0.1f}".format(BRW.sensor_reading)+chr(223)+"c ",1,0)
        mylcd.lcd_display_string("{:5d}".format(BRW.output)+"% BRW",1,6)
        mylcd.lcd_display_string(time.strftime("%A"),2,0)
        mylcd.lcd_display_string(time.strftime("%H:%M"),2,11)

        time.sleep(.3)

#        if verbose:
#            print(time.strftime("%H:%M:%S"), "{0:8.3f}".format(sensor_reading), "{0:8.0f}".format(BRW.output), "{0:8.1f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0),"{0:>8s}".format('Brew'))

    GPIO.output(24,GPIO.LOW) #  turn off pump
    boilerPWM.stop() #boiler off

def Steam():
    mylcd.lcd_clear()
    mylcd.lcd_display_string("STEAM",1,0)
    time.sleep(.5)

    max = time.time() + 900
    while GPIO.input(21):

        if time.time() > max:
            break

        sensor_reading=read_temp()
        STM.calc(sensor_reading)

        mylcd.lcd_display_string("{0:0.1f}".format(STM.sensor_reading)+chr(223)+"c ",1,7)
        mylcd.lcd_display_string("{:5d}".format(STM.output)+"% PWR",2,0)
        mylcd.lcd_display_string("STEAM",1,0)

        time.sleep(.3)

        if settings["verbose"]:
            print(time.strftime("%H:%M:%S"), "{0:8.3f}".format(sensor_reading), "{0:8.0f}".format(boost), "{0:8.1f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0),"{0:>8s}".format('Steam'))

    boilerPWM.stop() #boiler off
    mylcd.lcd_clear()
    time.sleep(.5)

def flush():
    mylcd.lcd_clear()
    mylcd.lcd_display_string("FLUSH",1,0)
    time.sleep(.5)
    GPIO.output(24,GPIO.HIGH)
    time.sleep(15)
    GPIO.output(24,GPIO.LOW)
    mylcd.lcd_clear()

def increment_setpoint(settings):
#    print("Button was pushed!")
    mylcd.lcd_clear()
    mylcd.lcd_display_string("setpoint: " + "{0:0.1f}".format(settings["setpoint"])+chr(223)+"c ",1,0)
    time.sleep(2)
    while GPIO.input(16) == GPIO.LOW:
        settings["setpoint"] +=.1
        if settings["setpoint"]>100:
            settings["setpoint"] = 94
        mylcd.lcd_display_string("setpoint: " + "{0:0.1f}".format(settings["setpoint"])+chr(223)+"c ",1,0)
        time.sleep(.3)

    for x in range(0, 8):
        mylcd.lcd_clear()
        time.sleep(.1)
        mylcd.lcd_display_string("setpoint: " + "{0:0.1f}".format(settings["setpoint"])+chr(223)+"c ",1,0)
        time.sleep(.1)

    file = open("settings2.txt","w+")
    file.write(json.dumps(settings))
    file.close()

#main
killer = GracefulKiller()

#GPIO.add_event_detect(16,GPIO.RISING,callback=button_callback) # Setup event on pin 10 rising edge

# the boiler control uses artifical pulse-width modulation of the Pi
# PIN 23 is assigned to a Fostek SSR wired to the boiler
# 50 HZ is the switch rate which should allow the SSR to switch on/off
# in line with the AC phases of the 220v mains current
boilerPWM=GPIO.PWM(23, 50)

# read config settings from JSON file

try:
    file = open("settings2.txt","r")
    x = file.read()
    file.close()
    settings = json.loads(x)
    timeout = time.time() + 60*settings["minutes"]

except FileNotFoundError:
    # if config file not found, make one

    settings = {
        "setpoint": 96.5,
        "integral": 0,
        "delta": 0,
        "derivative" : 0,
        "steam" : 130,
        "dt" : 1,
        "Kp" : 6,
        "Ki" : 0.06,
        "Kd" : 48,
        "antiwindup" : 2,
        "verbose" : False,
        "minutes" : 30,
    }

    # convert and write to JSON config file:
    file = open("settings2.txt","w+")
    file.write(json.dumps(settings))
    file.close()
    timeout = time.time() + 60*setting["minutes"]

# instantiate PID objects
PID=PIDController(settings["setpoint"], settings["antiwindup"], settings["Kp"], settings["Kd"], settings["Ki"], 'PID')
BRW=PIDController(settings["setpoint"]+2, settings["antiwindup"], 12, settings["Kd"], 0, 'BRW')
STM=PIDController(settings["steam"], settings["antiwindup"], settings["Kp"], settings["Kd"], settings["Ki"], 'STM')

print("Gaggia PID Started:", os.path.basename(__file__))
if settings["verbose"]:
    print("Press control-c or kill " +str(os.getpid())+ " to end.")
    print("setpoint:",settings["setpoint"],"\r\ndt:",settings["setpoint"],"\r\nKp:",settings["Kp"],"\r\nKi:",settings["Ki"],"\r\nKd:",settings["Kd"])
    print("Time","{0:>12s}".format("Temp"),"{0:>8s}".format("boiler"),"{0:>8s}".format("Out"),"{0:>8s}".format("P"),"{0:>8s}".format("I"),"{0:>8s}".format("D"),"{0:>8s}".format("action"))
else:
    print("terminal output suppressed.  Press control-c or kill " +str(os.getpid())+ " to end.")

#MAIN LOOP
while True:
    # get the temperature reading from the sensor
    sensor_reading = read_temp()
    # update the PID
    if time.time() < timeout:
        PID.calc(sensor_reading)
        mylcd.lcd_display_string("{0:0.1f}".format(PID.sensor_reading)+chr(223)+"c ",1,0)
        mylcd.lcd_display_string("{:5d}".format(PID.output)+"% PID",1,6)
        mylcd.lcd_display_string(time.strftime("%A"),2,0)
        mylcd.lcd_display_string(time.strftime("%H:%M"),2,11)
    else:
        boilerPWM.stop() #boiler off
        mylcd.lcd_display_string("{0:0.1f}".format(sensor_reading)+chr(223)+"c ",1,0)
        mylcd.lcd_display_string("    SLEEP",1,7)
        mylcd.lcd_display_string(time.strftime("%A"),2,0)
        mylcd.lcd_display_string(time.strftime("%H:%M"),2,11)
        if settings["verbose"]:
            print(time.strftime("%H:%M:%S"), "{0:8.3f}".format(sensor_reading), "{0:8.0f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0), "{0:8.1f}".format(0),"{0:>8s}".format('Sleep'))

    if GPIO.input(27):
        Brew()
        timeout = time.time() + 60*settings["minutes"]

    if GPIO.input(21) == GPIO.LOW:
        Steam()

    if GPIO.input(16) == GPIO.LOW:
        increment_setpoint(settings)

    if GPIO.input(12) == GPIO.LOW:
        flush()


    # this traps errors including keyboard interrupt to switch the boiler
    # off safely in case of exit or kill of the process

    if killer.kill_now:
        boilerPWM.stop()
        GPIO.cleanup() # this ensures a clean exit
        pi.stop()
        mylcd.lcd_clear()
        mylcd.lcd_display_string("--Program end--",1,0)
        print("---GPIOs released--")
        print("---Program end--")
        break
