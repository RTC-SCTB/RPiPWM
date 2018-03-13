#!/usr/bin/env python3
import RPiPWM
import time
import os
import psutil
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont

# создаем объект, который будет управлять ШИМ сигналами
pwm = RPiPWM.Pwm()
# номера каналов, куда какой объект будет подключен
chanOnOff = 0
chanSrv270 = 1
# драйверы обычно генерируют свои 5 вольт, которые могут вернуть на плату
# поэтому их стоит подключать к каналам 12 - 15
chanRevMotor = 12

# инициализируем каналы
pwm.InitChannel(chanOnOff, RPiPWM.PwmMode.onOff)
pwm.InitChannel(chanSrv270, RPiPWM.PwmMode.servo270)
pwm.InitChannel(chanRevMotor, RPiPWM.PwmMode.reverseMotor)

print("Initing channels: %d - On/Off, %d - Servo270, %d - Reverse Motor"
              % (chanOnOff, chanSrv270, chanRevMotor))

# будем циклично изменять значения на каналах от 0 до максимума, а потом обратно
onOff = False   # текущие значения для каналов
servo270 = 0
revMotor = 100

servo270Back = False  # флаги по которому будем определять что по диапазону пора идти обратно
revMotorBack = False

servo270Step = 90  # шаг с которым будем увеличивать/уменьшать значение на канале
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
    onOff = not onOff   # переключаем вкл/выкл

    if servo270Back is False:       # идем по диапазону от 0 до 270
        servo270 += servo270Step
        if servo270 >= 270:         # если дошли до конца диапазона
            servo270 = 270
            servo270Back = True     # ставим флаг, что надо идти обратно
    else:
        servo270 -= servo270Step    # аналогично, только идем по диапазону в обратную сторону
        if servo270 <= 0:
            servo270 = 0
            servo270Back = False

    if revMotorBack is False:
        revMotor += revMotorStep
        if revMotor > 100:
            revMotor = 100
            revMotorBack = True
    else:
        revMotor -= revMotorStep
        if revMotor < -100:
            revMotor = -100
            revMotorBack = False

    # задаем значения на каналах
    pwm.SetChannel(chanOnOff, onOff)
    pwm.SetChannel(chanSrv270, servo270)
    pwm.SetChannel(chanRevMotor, revMotor)
    print("Channel values: %d: %d\t%d: %d\t%d: %d"
          % (chanOnOff, onOff, chanSrv270, servo270, chanRevMotor, revMotor))

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