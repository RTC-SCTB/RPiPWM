import smbus as I2C
import RPi.GPIO as GPIO
import time
from enum import Enum   # для создания нумерованных списков
import math

###
'''
Класс для получения информации от одноканального АЦП MCP3221.
При инициализации задаются значения: vRef - опорное напряжение (относительно которого происходит измерение),
gain - коэффициент делителя напряжения (если он есть.
Методы:
Read - читает информацию из шины I2C (2 байта измерения);
GetVoltage - Вызывает метод Read, преобразует полученное значение в напряжение исходя из заданного опорного напряжения;
GetBattery - Вызывает метод GetVoltage, домножает полученное напряжение на коэффициент делителя напряжения.
'''
###


class Battery:
    def __init__(self, vRef=3.3, gain=7.66):
        self._addr = 0x4D
        self._vRef = vRef
        self._gain = gain
        self._i2c = I2C.SMBus(1)

    def _Read(self):
        reading = self._i2c.read_i2c_block_data(self._addr, 0x00, 2)
        return (reading[0] << 8) + reading[1]

    def _GetRefVoltage(self):
        voltage = (self._Read() / 4095) * self._vRef  # 4095 - число разрядов АЦП
        return voltage

    def GetVoltage(self):
        battery = self._GetRefVoltage() * self._gain
        return round(battery, 2)


# Регистры для работы с PCA9685
_PCA9685_ADDRESS    = 0x40
_MODE1              = 0x00
_MODE2              = 0x01
_SUBADR1            = 0x02
_SUBADR2            = 0x03
_SUBADR3            = 0x04
_PRESCALE           = 0xFE
_LED0_ON_L          = 0x06
_LED0_ON_H          = 0x07
_LED0_OFF_L         = 0x08
_LED0_OFF_H         = 0x09
_ALL_LED_ON_L       = 0xFA
_ALL_LED_ON_H       = 0xFB
_ALL_LED_OFF_L      = 0xFC
_ALL_LED_OFF_H      = 0xFD

# Биты для работы с PCA9685:
_RESTART            = 0x80   # при чтении возвращает свое состояние, при записи - разрешает или запрещает перезагрузку
_SLEEP              = 0x10   # режим энергосбережения (выключен внутренний осциллятор)
_ALLCALL            = 0x01   # PCA9685 будет отвечать на запрос всех устройств на шине
_INVRT              = 0x10   # инверсный или неинверсный выход сигнала на микросхеме
_OUTDRV             = 0x04   # способ подключения светодиодов (см. даташит, нам это вроде не надо)

# при частоте ШИМ 50 Гц (20 мс) получаем
_min = 205  # 1 мс (~ 4096/20)
_max = 410  # 2 мс (~ 4096*2/20)

###
'''
Класс для работы с миркосхемой, генерирующей ШИМ сигналы.
Позволяет задавать ШИМ сигнал отдельно для каждого канала. Канал требует инициализации.
Возможные значения для инициализации канала (берутся из соответствующего нумерованного списка):
servo90 - для серв с углом поворота 90 градусов;
servo180 - для серв с углом поворота 180 градусов;
servo270 - для серв с углом поворота 270 градусов;
reverseMotor - для подключения драйвера моторов, у которого крайние значения отвечают за разное направление вращения моторов.
forwardMotor - для подключения драйвера моторов, у которого только одно направление вращения (от 0 до 100)
'''
###


class PwmMode(Enum):    # список режимов работы
    servo90 = 90        # серва 90 градусов
    servo180 = 180      # серва 180 градусов
    servo270 = 270      # серва 270 градусов
    forwardMotor = 100  # мотор без реверса
    reverseMotor = 4    # мотор с реверсом


class Pwm:
    def __init__(self):
        self._i2c = I2C.SMBus(1)
        # инициализируем микросхему
        self._WriteByte(_MODE2, _OUTDRV)
        self._WriteByte(_MODE1, _ALLCALL)
        time.sleep(0.005)
        mode1 = self._ReadU8(_MODE1)    # читаем установленный режим
        mode1 = mode1 & ~_SLEEP     # будим
        self._WriteByte(_MODE1, mode1)
        time.sleep(0.005)
        self._SetPwmFreq(50)    # устанавливаем частоту сигнала 50 Гц
        # словарь, содержащий номера каналов и выставленный режим
        self.channel = {}

    def _WriteByte(self, register, value):  # запись 8битного значения в заданный регистр
        value = value & 0xFF
        self._i2c.write_byte_data(_PCA9685_ADDRESS, register, value)

    def _ReadU8(self, register):    # чтение unsigned byte
        result = self._i2c.read_byte_data(_PCA9685_ADDRESS, register) & 0xFF
        return result

    def _SetPwmFreq(self, freqHz):  # устанавливает частоту ШИМ сигнала в Гц
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= freqHz
        prescaleval -= 1
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self._ReadU8(_MODE1)  # смотрим какой режим был у микросхемы
        newmode = (oldmode & 0x7F) | 0x10   # отключаем внутреннее тактирование, чтобы внести изменения
        self._WriteByte(_MODE1, newmode)
        self._WriteByte(_PRESCALE, prescale)    # изменяем частоту
        self._WriteByte(_MODE1, oldmode)    # включаем тактирование обратно
        time.sleep(0.005)   # ждем пока оно включится
        self._WriteByte(_MODE1, oldmode | 0x08)     # разрешаем микросхеме отвечать на subaddress 1

    def _SetPwm(self, channel, value):  # установка значения для канала
        self._WriteByte(_LED0_ON_L + 4 * channel, 0 & 0xFF)  # момент включения в цикле
        self._WriteByte(_LED0_ON_H + 4 * channel, 0 >> 8)
        self._WriteByte(_LED0_OFF_L + 4 * channel, value & 0xFF)  # момент выключения в цикле
        self._WriteByte(_LED0_OFF_H + 4 * channel, value >> 8)

    def Reset(self):    # программный сброс микросхемы
        val = 0x06 & 0xFF
        self._i2c.write_byte(_PCA9685_ADDRESS, val)

    def InitChannel(self, channel, value):
        self.channel[channel] = value

    def SetChannel(self, channel, value):
        try:
            mode = self.channel[channel]
        except KeyError:
            print("Channel haven't been inited!")
            return
        # если канал установлен на режим работы "в одну сторону" - сервы или мотор без реверса
        # то нужно устанавливать значение от 0 до максимального, задаваемого режимом
        if mode != PwmMode.reverseMotor:
            if value < 0: value = 0     # обрезаем крайние значения
            if value > mode: value = mode
            value *= 205/mode   # изменяем диапазон 0-mode -> 0-205
            value += 205        # сдвигаем диапазон 0-205 -> 205-410
        else:   # если говорим о моторе с реверсом
            if value < -100: value = -100   # обрезаем диапазон
            if value > 100: value = 100
            value += 100    # сдвигаем диапазон -100-100 -> 0-200
            value *= 205/200    # чуть изменяем 0-200 -> 0-205
            value += 205    # сдвигаем 0-205 -> 205-410
        self._SetPwm(channel, int(value))  # устанавливаем значение




###
'''
Класс для работы с кнопкой и светодиодом.
При инициализации указывается на каком пине нужно слушать кнопку, на каком - висит светодиод.
'''
###
chanButton = 20
chanLed = 21


class Gpio:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(chanButton, GPIO.IN, pull_up_down = GPIO.OFF)
        GPIO.setup(chanLed, GPIO.OUT, initial=GPIO.LOW)

    def ButtonAddEvent(self, foo):    # добавление функции, которая срабатывает при нажатии на кнопку
        if foo is not None:
            GPIO.add_event_detect(20, GPIO.FALLING, callback = foo, bouncetime = 200)

    def LedSet(self, value):    # включает или выключает светодиод в зависимости от заданного значения
        GPIO.output(chanLed, value)

    def LedToggle(self):    # переключает состояние светодиода
        GPIO.output(chanLed, not GPIO.input(chanLed))

    def CleanUp(self):
        GPIO.cleanup()


