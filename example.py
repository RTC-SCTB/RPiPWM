#!/usr/bin/env python3
import RPiPWM
import time
import os
import psutil
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont

# номера каналов, куда какой объект будет подключен
chanOnOff = 0
chanSrv180 = 1
# драйверы обычно генерируют свои 5 вольт, которые могут вернуть на плату
# поэтому их стоит подключать к каналам 12 - 15
chanRevMotor = 2

# создаем объекты для образца
switch = RPiPWM.Switch(chanOnOff)       # на этом канале будут просто чередоваться высокий и низкий уровни
servo = RPiPWM.Servo180(chanSrv180, extended=True)     # серва 180 градусов, почему-то моей потребовался широкий диапазон
motor = RPiPWM.ReverseMotor(chanRevMotor)   # мотор с реверсом

print("Initing channels: %d - On/Off, %d - Servo180, %d - Reverse Motor"
      % (chanOnOff, chanSrv180, chanRevMotor))

# будем циклично изменять значения на каналах от 0 до максимума, а потом обратно
switchState = False   # текущие значения для каналов
servoValue = 0
motorValue = 100

servoBack = False  # флаги по которому будем определять что по диапазону пора идти обратно
revMotorBack = False

servoStep = 45  # шаг с которым будем увеличивать/уменьшать значение на канале
revMotorStep = 50

# создаем объект, который будет работать с АЦП
# указываем опорное напряжение, оно замеряется на первом пине Raspberry (обведено квадратом на шелкографии)
adc = RPiPWM.Battery(vRef=3.28)
adc.start()     # запускаем измерения

# создаем объект для работы с дисплеем (еще возможные варианты - 128_32 и 96_16 - размеры дисплеев в пикселях)
disp = RPiPWM.SSD1306_128_64()
disp.Begin()    # запускаем дисплей
disp.Clear()    # очищаем буффер изображения
disp.Display()  # выводим пустую картинку на дисплей

width = disp.width  # получаем высоту и ширину дисплея
height = disp.height

image = Image.new('1', (width, height))     # создаем изображение из библиотеки PIL для вывода на экран
draw = ImageDraw.Draw(image)    # создаем объект, которым будем рисовать
top = -2    # сдвигаем текст вверх на 2 пикселя
x = 0   # сдвигаем весь текст к левому краю
font = ImageFont.load_default()     # загружаем стандартный шрифт


# функция, которая будет срабатывать при нажатии на кнопку
def ButtonEvent(a):     # обязательно должна иметь один аргумент
    print("Somebody pressed button!")


# создаем объект для работы с кнопкой и светодиодом
gpio = RPiPWM.Gpio()
gpio.ButtonAddEvent(ButtonEvent)    # связываем нажатие на кнопку с функцией

while True:
    switchState = not switchState   # переключаем вкл/выкл

    if servoBack is False:       # идем по диапазону от 0 до 270
        servoValue += servoStep
        if servoValue >= 180:         # если дошли до конца диапазона
            servoValue = 180
            servoBack = True     # ставим флаг, что надо идти обратно
    else:
        servoValue -= servoStep    # аналогично, только идем по диапазону в обратную сторону
        if servoValue <= 0:
            servoValue = 0
            servoBack = False

    if revMotorBack is False:
        motorValue += revMotorStep
        if motorValue > 100:
            motorValue = 100
            revMotorBack = True
    else:
        motorValue -= revMotorStep
        if motorValue < -100:
            motorValue = -100
            revMotorBack = False
    print("Old channel values: %d: %d\t%d: %d\t%d: %d"
          % (chanOnOff, switch.GetValue(), chanSrv180, servo.GetValue(), chanRevMotor, motor.GetValue()))
    # задаем значения на каналах
    servo.SetValue(servoValue)
    switch.SetValue(switchState)
    motor.SetValue(motorValue)
    print("New channel values: %d: %d\t%d: %d\t%d: %d"
          % (chanOnOff, switchState, chanSrv180, servoValue, chanRevMotor, motorValue))
    print()     # пустая строчка
    voltage = adc.GetVoltageFiltered()  # получаем напряжение аккумулятора

    draw.rectangle((0, 0, width, height), outline=0, fill=0)  # прямоугольник, залитый черным - очищаем дисплей
    draw.text((x, top), "Some interesting info", font=font, fill=255)        # формируем текст
    draw.text((x, top + 8), "Battery: "+str(voltage)+ " V", font=font, fill=255)     # высота строки - 8 пикселей
    draw.text((x, top + 16), "Only english :(", font=font, fill=255)
    draw.text((x, top + 24), "And 21 symbol", font=font, fill=255)

    disp.Image(image)   # записываем изображение в буффер
    disp.Display()      # выводим его на экран

    gpio.LedToggle()    # переключаем светодиод

    time.sleep(1)