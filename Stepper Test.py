from machine import Pin, Signal, PWM
from time import sleep_ms, ticks_ms

FL_EN = Signal(Pin(11,Pin.OUT), invert = True)
FR_EN = Signal(Pin( 5,Pin.OUT), invert = True)
BL_EN = Signal(Pin( 8,Pin.OUT), invert = True)
BR_EN = Signal(Pin( 2,Pin.OUT), invert = True)

FL_EN.off()
FR_EN.off()
BL_EN.off()
BR_EN.off()

FL_DIR = Pin(9,Pin.OUT)
FR_DIR = Pin(3,Pin.OUT)
BL_DIR = Pin(6,Pin.OUT)
BR_DIR = Pin(0,Pin.OUT)

FL_DIR.value(0)
FR_DIR.value(1)
BL_DIR.value(0)
BR_DIR.value(1)

FL_STEP = PWM(Pin(10,Pin.OUT))
FR_STEP = PWM(Pin( 4,Pin.OUT))
BL_STEP = PWM(Pin( 7,Pin.OUT))
BR_STEP = PWM(Pin( 1,Pin.OUT))

FL_STEP.freq(50)
FR_STEP.freq(50)
BL_STEP.freq(50)
BR_STEP.freq(50)

FL_STEP.duty_u16(50)
FR_STEP.duty_u16(50)
BL_STEP.duty_u16(50)
BR_STEP.duty_u16(50)

FL_VREF = PWM(Pin(15,Pin.OUT))
FR_VREF = PWM(Pin(13,Pin.OUT))
BL_VREF = PWM(Pin(14,Pin.OUT))
BR_VREF = PWM(Pin(12,Pin.OUT))

FL_VREF.freq(100000)
FR_VREF.freq(100000)
BL_VREF.freq(100000)
BR_VREF.freq(100000)

IlimitDuty = 65 * 750          # value dervived from measurements  (250~1000mA)

FL_VREF.duty_u16(IlimitDuty)
FR_VREF.duty_u16(IlimitDuty)
BL_VREF.duty_u16(IlimitDuty)
BR_VREF.duty_u16(IlimitDuty)

S1 = Pin(26,Pin.OUT)
S2 = Pin(27,Pin.OUT)
S3 = Pin(28,Pin.OUT)
S4 = Pin(29,Pin.OUT)

import neopixel

numPixels = 1
NeoPin = Pin(16,Pin.OUT)        
Neo = neopixel.NeoPixel(NeoPin,numPixels)
Neo[0] = (10,0,0)
neopixel.NeoPixel.write(Neo)

FL_EN.on()
FR_EN.on()
BL_EN.on()
BR_EN.on()

sleep_ms(10000)

FL_EN.off()
FR_EN.off()
BL_EN.off()
BR_EN.off()