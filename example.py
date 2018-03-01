#!/usr/bin/env python3
import RPiPWM
import threading
import time
import subprocess
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont

exit = False

disp = RPiPWM.SSD1306_128_64()
disp.Begin()
disp.Clear()
disp.Display()

width = disp.width  # берем ширину и высоту из дисплея чтобы все точно совпадало
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)    # создаем объект с помощью которого будем рисовать изображение для дисплея

draw.rectangle((0, 0, width, height), outline=0, fill=0)    # прямоугольник, залитый черным чтобы очистить изображение
# несколько переменных для удобства рисования
padding = -2    # отступы
top = padding
bottom = height-padding
# двигаемся слева на право, отслеживаем текущую позицию по x
x = 0
font = ImageFont.load_default()  # загружаем стандартный шрифт
# можно подгрузить шрифт в формате ttf, если он лежит в той же папке
# font = ImageFont.truetype('BlaBla.ttf', 8)
while True:
    # очищаем изображение
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    # Shell скрипты чтобы получить информацию о системе. Потом будем использовать библиотеку Psuils
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell = True )
    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    MemUsage = subprocess.check_output(cmd, shell = True )
    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    Disk = subprocess.check_output(cmd, shell = True )

    # Выводим строки текста
    draw.text((x, top), "IP: " + str(IP), font=font, fill=255)
    draw.text((x, top+8), str(CPU), font=font, fill=255)
    draw.text((x, top+16), str(MemUsage), font=font, fill=255)
    draw.text((x, top+25), str(Disk), font=font, fill=255)

    # Выводим изображение
    disp.Image(image)
    disp.Display()
    time.sleep(1)
# TODO: написать пример, который будет работать со всеми элементами библиотеки сразу

# pwm = RPiPWM.Pwm()
# pwm.InitChannel(0, RPiPWM.PwmMode.servo90)
#
# def ServoMover():
#     while not exit:
#         for i in range(90):
#             pwm.SetChannel(0, i)
#             time.sleep(0.01)
#         for i in range(90):
#             pwm.SetChannel(0, 90-i)
#             time.sleep(0.01)
#
#
# adc = RPiPWM.Battery(vRef=3.28, gain=7.66)
# print("Starting ADC")
# adc.start()
# value = 8.65
# print("Calibrating ADC with value %d" % value)
# adc.Calibrate(value)
# print("ADC started")
#
# gpio = RPiPWM.Gpio()
#
#
# def Informator():
#     while not exit:
#         print("Voltage: %.2f" % adc.GetVoltageFiltered())
#         time.sleep(1)
#     gpio.LedToggle()
#
# gpio.ButtonAddEvent(Informator)
#
# t1 = threading.Thread(target=Informator)
# t1.start()
# t1.join()
# time.sleep(5)
# exit = True