#!/usr/bin/env python3
import RPiPWM
import time
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont

# номера каналов, куда какой объект будет подключен
chanOnOff = 0
chanSrv180 = 1
chanSrv270 = 2
# драйверы обычно генерируют свои 5 вольт, которые могут вернуть на плату
# поэтому их стоит подключать к каналам 12 - 15
chanRevMotor = 12

# создаем объекты для образца
switch = RPiPWM.Switch(chanOnOff)       # на этом канале будут просто чередоваться высокий и низкий уровни
servo180 = RPiPWM.Servo180(chanSrv180, extended=True)
servo270 = RPiPWM.Servo270(chanSrv270, extended=True)   # серва 270 градусов, почему-то моей потребовался широкий диапазон
motor = RPiPWM.ReverseMotor(chanRevMotor)   # мотор с реверсом

print("Initing channels: %d - On/Off, %d - Servo180, %d - Servo270, %d - Reverse Motor"
      % (chanOnOff, chanSrv180, chanSrv270, chanRevMotor))

# будем циклично изменять значения на каналах от 0 до максимума, а потом обратно
switchState = False   # текущие значения для каналов
servo180Value = 0
servo270Value = 0
motorValue = 100

servo180Back = False  # флаги по которому будем определять что по диапазону пора идти обратно
servo270Back = False
revMotorBack = False

servoStep = 10  # шаг с которым будем увеличивать/уменьшать значение на канале
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

    if servo180Back is False:       # идем по диапазону от 0 до 180
        servo180Value += servoStep
        if servo180Value >= 180:         # если дошли до конца диапазона
            servo180Value = 180
            servo180Back = True     # ставим флаг, что надо идти обратно
    else:
        servo180Value -= servoStep    # аналогично, только идем по диапазону в обратную сторону
        if servo180Value <= 0:
            servo180Value = 0
            servo180Back = False

    if servo270Back is False:
        servo270Value += servoStep
        if servo270Value >= 270:
            servo270Value = 270
            servo270Back = True
    else:
        servo270Value -= servoStep
        if servo270Value <= 0:
            servo270Value = 0
            servo270Back = False


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
    # задаем значения на каналах
    servo180.setValue(servo180Value)
    servo270.setValue(servo270Value)
    switch.setValue(switchState)
    motor.setValue(motorValue)
    print("Channel %d:\t%d val,\t%.2f ms"
          % (chanSrv180, servo180.getValue(), servo180.getMcs()))
    voltage = adc.getVoltageFiltered()  # получаем напряжение аккумулятора

    draw.rectangle((0, 0, width, height), outline=0, fill=0)  # прямоугольник, залитый черным - очищаем дисплей
    draw.text((x, top), "Some interesting info", font=font, fill=255)        # формируем текст
    draw.text((x, top + 8), "Battery: "+str(voltage)+ " V", font=font, fill=255)     # высота строки - 8 пикселей
    draw.text((x, top + 16), "Only english :(", font=font, fill=255)
    draw.text((x, top + 24), "And 21 symbol", font=font, fill=255)

    disp.Image(image)   # записываем изображение в буффер
    disp.Display()      # выводим его на экран

    gpio.LedToggle()    # переключаем светодиод

    time.sleep(1)