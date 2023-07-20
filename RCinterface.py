###############################################################################
# Roverling - RC Interface (mine: Spektrum AR6200 Rx, DX6i Tx)
# Interrupt on both edges to mark tick_us points, process later at a reduced
# rate, as required, to keep these IRQ routines as short as possible
#
# v1.01 20-07-2023 Starting 

from machine import Pin
from time import ticks_us

ch1 = Pin(27,Pin.IN)
ch2 = Pin(13,Pin.IN)
ch3 = Pin(14,Pin.IN)
ch4 = Pin(28,Pin.IN)
ch5 = Pin(15,Pin.IN)
ch6 = Pin(26,Pin.IN)

tt = ticks_us()
CHtimes = [[tt,tt], [tt,tt], [tt,tt], [tt,tt], [tt,tt], [tt,tt]]

def cbIntCh1(ch1):
    global CHtimes
    if ch1.value() == 1:
        CHtimes[0][0] = ticks_us()
    else:
        CHtimes[0][1] = ticks_us()

def cbIntCh2(ch2):
    global CHtimes
    if ch2.value() == 1:
        CHtimes[1][0] = ticks_us()
    else:
        CHtimes[1][1] = ticks_us()

def cbIntCh3(ch3):
    global CHtimes
    if ch3.value() == 1:
        CHtimes[2][0] = ticks_us()
    else:
        CHtimes[2][1] = ticks_us()

def cbIntCh4(ch4):
    global CHtimes
    if ch4.value() == 1:
        CHtimes[3][0] = ticks_us()
    else:
        CHtimes[3][1] = ticks_us()

def cbIntCh5(ch5):
    global CHtimes
    if ch5.value() == 1:
        CHtimes[4][0] = ticks_us()
    else:
        CHtimes[4][1] = ticks_us()

def cbIntCh6(ch6):
    global CHtimes
    if ch6.value() == 1:
        CHtimes[5][0] = ticks_us()
    else:
        CHtimes[5][1] = ticks_us()

ch1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=cbIntCh1)
ch2.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=cbIntCh2)
ch3.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=cbIntCh3)
ch4.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=cbIntCh4)
ch5.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=cbIntCh5)
ch6.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=cbIntCh6)

LastRC = [0,0,0,0,0,0]
RCavg = [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]

def GetRC(ch):
    # get raw times, convert to %, limit: 0 ~ 100
    # too reduce gitter (some caused by other interrupts) use a 5 element median filter
    global LastRC
    global RCavg
    if CHtimes[ch-1][1] > CHtimes[ch-1][0]:     # in case we sample mid pulse, ignore
        NewVal = int(min(max(((CHtimes[ch-1][1] - CHtimes[ch-1][0]) - 1000) / 10, 0), 100))
        RCavg[ch-1].append(NewVal)
        RCavg[ch-1].pop(0)                  # use median of last 5 samples
        tmpList = []
        tmpList.extend(RCavg[ch-1])
        tmpList.sort()
        LastRC[ch-1] = tmpList[2]           # centre position implies 2 values above and 2 below
    return LastRC[ch-1]

