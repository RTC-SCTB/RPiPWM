#!/usr/bin/env python3
import RPiPWM
import threading
import time
import os
import psutil
from PIL import Image       # библиотеки для рисования на дисплее
from PIL import ImageDraw
from PIL import ImageFont


class Pwm(threading.Thread):    # класс, показывающий работу с каналами ШИМ
    def __init__(self):
        threading.Thread.__init__(self)
        self.pwm = RPiPWM.Pwm()
        self.chanOnOff = 0
        self.chanSrv270 = 1
        self.chanRevMotor = 2
        self.pwm.InitChannel(self.chanOnOff, RPiPWM.PwmMode.onOff)
        self.pwm.InitChannel(self.chanSrv270, RPiPWM.PwmMode.servo270)
        self.pwm.InitChannel(self.chanRevMotor, RPiPWM.PwmMode.reverseMotor)
        print("Initing channels: %d - On/Off, %d - Servo270, %d - Reverse Motor"
              % (self.chanOnOff, self.chanSrv270, self.chanRevMotor))
        self.exit = False

    def run(self):
        onOff = False
        servo270 = 0
        servo270Back = False    # флаг по которому будем определять что по диапазону пора идти обратно
        servo270Step = 90   # шаг с которым будем увеличивать/уменьшать значение на канале
        revMotor = 100
        revMotorBack = False
        revMotorStep = 50
        print("Starting pwm channels")
        while not self.exit:
            if onOff:   # переключаем канал вкл/выкл
                onOff = False
            else:
                onOff = True

            if servo270Back is True:    # шагаем по диапазону servo270 от 0 до 270 в одну сторону
                servo270 -= servo270Step
                if servo270 <= 0:
                    servo270 = 0
                    servo270Back = False
            else:                       # и в другую сторону
                servo270 += servo270Step
                if servo270 >= 270:
                    servo270 = 270
                    servo270Back = True

            if revMotorBack is True:    # аналогично, но по диапазону rev motor от -100 до 100
                revMotor -= revMotorStep
                if revMotor < -100:
                    revMotor = -100
                    revMotorBack = False
            else:
                revMotor += revMotorStep
                if revMotor > 100:
                    revMotor = 100
                    revMotorBack = True
            print("Channel values: %d: %d\t%d: %d\t%d: %d"
                  % (self.chanOnOff, onOff, self.chanSrv270, servo270, self.chanRevMotor, revMotor))
            self.pwm.SetChannel(self.chanOnOff, onOff)
            self.pwm.SetChannel(self.chanSrv270, servo270)
            self.pwm.SetChannel(self.chanRevMotor, revMotor)
            time.sleep(1)
        print("Pwm channels stopped")

    def Stop(self):
        self.exit = True
        print("Stopping thread")


class Informer(threading.Thread):   # класс выводящий информацию на дисплей
    def __init__(self):
        threading.Thread.__init__(self)
        print("Creating display")
        self.disp = RPiPWM.SSD1306_128_64()     # создаем дисплей
        self.disp.Begin()       # инициализируем его
        self.disp.Clear()       # очищаем
        self.disp.Display()     # выводим пустой кадр
        print("Creating ADC")
        self.adc = RPiPWM.Battery(vRef=3.28, gain=7.66)     # создаем объект, получающий показания с АЦП
        self.exit = False       # флаг для завершения тредов

        self.gpio = RPiPWM.Gpio()   # класс для работы с кнопкой и светодиодом
        self.gpio.ButtonAddEvent(self.Button)   # привязываем к нажатию на кнопку функцию

    def GetCpuTemperature(self):    # функция для получения температуры процессора
        res = os.popen('vcgencmd measure_temp').readline()
        return float(res.replace('temp=', '').replace('\'C\n', ''))

    def GetCpuLoad(self):   # получаем загрузку процессора в %
        res = psutil.cpu_percent()
        return res

    def GetIP(self):    # функция для получения собственного ip адреса
        res = os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n', '')  # получаем IP, удаляем \n
        return res

    def GetBattery(self):   # функция для получения текущего значения с АЦП
        return self.adc.GetVoltageFiltered()

    def Button(self, a):    # обязательно нужен один аргумент
        print("Somebody pressed button!")

    def run(self):
        print("Starting ADC")
        self.adc.start()    # запускаем объект АЦП, чтобы он отслеживал уровень заряда батарейки и фильтровал значения
        width = self.disp.width
        height = self.disp.height
        image = Image.new('1', (width, height))     # создаем изображение
        draw = ImageDraw.Draw(image)    # создаем объект с помощью которого будем рисовать
        draw.rectangle((0,0,width,height), outline=0, fill=0)   # прямоугольник, залитый черным чтобы очистить изображение
        # несколько переменных для удобства рисования
        padding = -2  # отступы
        top = padding
        x = 0   # сдвиг для всех строчек слева
        font = ImageFont.load_default()  # загружаем стандартный шрифт
        # можно подгрузить шрифт в формате ttf, если он лежит в той же папке
        # font = ImageFont.truetype('BlaBla.ttf', 8)
        IP = self.GetIP()   # получаем информацию об IP адресе
        print("Starting display")
        while not self.exit:
            # очищаем изображение
            draw.rectangle((0, 0, width, height), outline=0, fill=0)
            # получаем сведения о системе
            cpuTemp = str(self.GetCpuTemperature())
            cpuLoad = str(self.GetCpuLoad())
            battery = str(self.GetBattery())
            # формируем картинку для дисплея
            draw.text((x, top), "IP: "+IP, font=font, fill=255)
            draw.text((x, top + 8), "CPU: "+cpuLoad+" %, "+cpuTemp+"°C", font=font, fill=255)
            draw.text((x, top + 16), "Battery: "+battery+" V", font=font, fill=255)
            # выводим изображение
            self.disp.Image(image)
            self.disp.Display()

            self.gpio.LedToggle()   # переключаем светодиод
            time.sleep(1)
        self.adc.Stop()
        print("ADC and Display stopped")

    def Stop(self):
        self.exit = True

info = Informer()
pwm = Pwm()
info.start()
pwm.start()
info.join()
pwm.join()
