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


adc = RPiPWM.Battery()

gpio = RPiPWM.Gpio()

def Informator(a):
    print("Voltage: %f" % adc.GetVoltage())
    gpio.LedToggle()


gpio.ButtonAddEvent(Informator)

t1 = threading.Thread(target=ServoMover)
t1.start()
t1.join()
time.sleep(5)
exit = True