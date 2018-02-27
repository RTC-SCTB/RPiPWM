import RPiPWM


def HelloWorld():
    print("Hello world!")


pwm = RPiPWM.Pwm()
pwm.InitChannel(1, RPiPWM.PwmMode.servo90)


adc = RPiPWM.Battery()
adc.GetVoltage()

gpio = RPiPWM.Gpio()
gpio.LedToggle()
gpio.ButtonAddEvent(HelloWorld)



