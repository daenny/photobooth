#!/usr/bin/env python
# Created by br _at_ re-web _dot_ eu, 2015

try:
    import RPi.GPIO as GPIO
    gpio_enabled = True
except ImportError:
    gpio_enabled = False

import pygame
try:
    import pygame.fastevent as EventModule
except ImportError:
    import pygame.event as EventModule


class Event:
    def __init__(self, type, value):
        """type  0: quit
                 1: keystroke 
                 2: mouseclick
                 3: gpio
        """
        self.type = type
        self.value = value

import serial
import threading
import glob

def trigger_event(event_channel):
    EventModule.post(EventModule.Event(pygame.USEREVENT, channel=event_channel))


class ArduinoSerial(threading.Thread):
    def __init__(self):
        super(ArduinoSerial, self).__init__()
        l = glob.glob('/dev/cu.wchu*')
        self.arduino = serial.Serial(l[0], 9600)

        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def run(self):
        while not self.stopped():
            try:
                if self.arduino.inWaiting():
                    line = self.arduino.readline()
                    if "KEY" in line:
                        trigger_event("ArduinoButton")
                        print "ButtonPressed"

            except Exception as e:
                print("Error: %s"%e)
                self.arduino = serial.Serial('/dev/cu.wchusbserial1410', 9600)


class Rpi_GPIO:
    def __init__(self, handle_function, input_channels = [], output_channels = []):
        if gpio_enabled:
            # Display initial information
            print("Your Raspberry Pi is board revision " + str(GPIO.RPI_INFO['P1_REVISION']))
            print("RPi.GPIO version is " + str(GPIO.VERSION))

            # Choose BCM numbering system
            GPIO.setmode(GPIO.BCM)

            # Setup the input channels
            for input_channel in input_channels:
                GPIO.setup(input_channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                GPIO.add_event_detect(input_channel, GPIO.RISING, callback=handle_function, bouncetime=200)

            # Setup the output channels
            for output_channel in output_channels:
                GPIO.setup(output_channel, GPIO.OUT)
                GPIO.output(output_channel, GPIO.LOW)
        else:
            print("Warning: RPi.GPIO could not be loaded. GPIO disabled.")

    def teardown(self):
        if gpio_enabled:
            GPIO.cleanup()

    def set_output(self, channel, value=0):
        if gpio_enabled:
            GPIO.output(channel, GPIO.HIGH if value==1 else GPIO.LOW)
