#!/usr/bin/env python3
import RPiPWM
import threading
import time


exit = False

pwm = RPiPWM.Pwm()
pwm.InitChannel(0, RPiPWM.PwmMode.servo90)

def ServoMover():
    while not exit:
        for i in range(90):
            pwm.SetChannel(0, i)
            time.sleep(0.01)
        for i in range(90):
            pwm.SetChannel(0, 90-i)
            time.sleep(0.01)


adc = RPiPWM.Battery(vRef=3.28, gain=7.66)
print("Starting ADC")
adc.start()
value = 8.65
print("Calibrating ADC with value %d" % value)
# adc.Calibrate(value)
print("ADC started")

gpio = RPiPWM.Gpio()


def Informator():
    while not exit:
        print("Voltage: %.2f" % adc.GetVoltageFiltered())
        time.sleep(1)
    # gpio.LedToggle()

gpio.ButtonAddEvent(Informator)

t1 = threading.Thread(target=Informator)
t1.start()
t1.join()
time.sleep(5)
exit = True