###############################################################################
# Roverling Motor Control Module
#
# v1.01 15-07-2023 Starting 
# v1.02 20-07-2023 Adding RC control 

from machine import Pin, Signal, PWM, Timer
from time import sleep_ms, ticks_ms, ticks_us
from math import pi
import neopixel

# Stepper Driver Control Pins
#  [FL] 0 ----- 1 [FR]
#    |              |
#    |              |
#    |              |
#  [BL] 2 ----- 3 [BR]

mEN =  [Signal(Pin(11,Pin.OUT), invert = True),
        Signal(Pin( 5,Pin.OUT), invert = True),
        Signal(Pin( 8,Pin.OUT), invert = True),
        Signal(Pin( 2,Pin.OUT), invert = True)]
for i in mEN: i.off()   # disable motor drivers         

mDIR = [Signal(Pin(9,Pin.OUT), invert = False),
        Signal(Pin(3,Pin.OUT), invert = True),
        Signal(Pin(6,Pin.OUT), invert = False),
        Signal(Pin(0,Pin.OUT), invert = True)]
for i in mDIR: i.off()  # direction forward 

mSTEP = [PWM(Pin(10,Pin.OUT)),
         PWM(Pin( 4,Pin.OUT)),
         PWM(Pin( 7,Pin.OUT)),
         PWM(Pin( 1,Pin.OUT))]
for i in mSTEP: 
    i.duty_u16(0)  # FIXED at 50% when running, 0% to hold
    i.freq(50)      # sets steps per second

StepsPerM = int(1 / (pi * 0.12 ) * 200) # 200 steps per rev.  120mm diam 

mVREF = [PWM(Pin(15,Pin.OUT)),
        PWM(Pin(13,Pin.OUT)),
        PWM(Pin(14,Pin.OUT)),
        PWM(Pin(12,Pin.OUT))]
for i in mVREF: 
    i.freq(100000)  # FIXED at 100kHz
    i.duty_u16(0)  

# Indicators
import neopixel
numPixels = 1
NeoPin = Pin(16,Pin.OUT)        
Neo = neopixel.NeoPixel(NeoPin,numPixels)
Neo[0] = (10,0,0)
neopixel.NeoPixel.write(Neo)

#####################################
# Motor Control
# 17/7/2023 With platform fully loaded, level surface, stall recovery  at:
# 2WD(F)        750mA     0.25 m/s  (132Hz)
# 4WD           500mA     0.25 m/s  
# 4WD           1000mA    0.60 m/s  (318Hz)
# crazy tests
# 1WD(no load)  1000mA    16.5 m/s  (accel 0.25 m/s/s) (8745Hz) (60km/hr)!!    
# 4WD(no load)  1000mA    5.0  m/s  (accel 0.1 m/s/s) vibrations rule

# computed velocity/PWM for Current, Target and required Step 
Vcur = [0, 0, 0, 0]
Vtarget = [0, 0, 0, 0]
Vstep = [0, 0, 0, 0]
PWMcur = [0, 0, 0, 0]
PWMstep = [0, 0, 0, 0]
PWMtarget = [0, 0, 0, 0]

# set velocity target and acceleration steps
def SetVel(i, vel, accel):
    global Vstep
    global Vtarget
    global PWMstep
    global PWMtarget

    Vtarget[i] = vel
    if accel != 0 and Vtarget[i] != Vcur[i]:       
        TimeToTarget = abs((Vtarget[i] - Vcur[i]) / accel)
        NumStepsReqd = (1000 / MotionPeriod) * TimeToTarget      
        Vstep[i] = (Vtarget[i] - Vcur[i]) / NumStepsReqd    # delta V per period

        PWMtarget[i] = int(Vtarget[i] * StepsPerM)
        PWMstep[i] = int(StepsPerM * Vstep[i])    # delta signed PWM per period

        #print('TimeToTarget', TimeToTarget, 'NumStepsReqd', NumStepsReqd)
        #print(i, 'Vcur', Vcur[i], 'Vtarget', Vtarget[i], 'Vstep', Vstep[i], 
        #      'PWMstep', PWMstep[i], 'PWMtarget', PWMtarget[i])

def SetCurrentmA(mA):
    #  Current Limit in mA.  250mA ~ 1000mA  multiplier = 65   
    for i in mVREF: 
        i.duty_u16(65 * max(min(mA,1000),250))

# at a regular interval increase velocity until target reached
def cbMotionTimer(MotionTimer):
    global PWMcur
    for i in range(4):
        if PWMstep[i] != 0 :
            PWMcur[i] = PWMcur[i] + PWMstep[i]
            
            # Target Reached
            if ((PWMstep[i] > 0 and PWMcur[i] > PWMtarget[i])     # pos accel
              or (PWMstep[i] < 0 and PWMcur[i] < PWMtarget[i])):  # neg accel
                    PWMcur[i] = PWMtarget[i]
                    PWMstep[i] = 0
                    #print('target on ', i, ' reached','PWM: ', mSTEP[i].freq())                    
                    if PWMcur[i] == 0:
                        mSTEP[i].duty_u16(0)    # HOLD when vely reaches zero
                        Vcur[i] = 0

            # Fix for illegal PWM freq
            if abs(PWMcur[i]) < 8:          # PWM freq must be greater than 8Hz
                mSTEP[i].freq(8)
            # or set new PWM freq & DIR
            else:
                if PWMcur[i] >= 0:          # set DIR according to sign of PWM
                    mDIR[i].off()
                else:
                    mDIR[i].on()

                mSTEP[i].freq(abs(PWMcur[i]))   # remove PWM sign
                mSTEP[i].duty_u16(50)           # RUN, testing shows 50% best 
                
                Vcur[i] = PWMcur[i] / StepsPerM # update current velocity

            #if i == 0: print('freq',mSTEP[i].freq(),'duty',mSTEP[i].duty_u16())

################################################################################

# Start RC Interface
from RCinterface import GetRC
sleep_ms(200)   # allow time for sample collection

# Use this first to set RC endpoints and subtrim
#while True:
#    sleep_ms(50)
#    print(GetRC(1),GetRC(2),GetRC(3),GetRC(4),GetRC(5),GetRC(6))

# Start locomotion processing
MotionTimer = Timer()
MotionPeriod = 100
MotionTimer.init(period=MotionPeriod, mode=Timer.PERIODIC, callback=cbMotionTimer)
sleep_ms(200) 

# enable stepper drivers 
SetCurrentmA(1000)  
for i in range(4): mEN[i].on()  

RCmaxVel = 2.0
RCaccel = 0.6

while True:
    sleep_ms(100)

    if GetRC(6) > 50:
        for i in range(4): mEN[i].off()
    else:
        for i in range(4): mEN[i].on()

    midVel = RCmaxVel * (GetRC(1) / 100)
    offset = RCmaxVel * (GetRC(2) - 50) / 100     # range -0.5 ~ +0.5
    
    if midVel != 0:    
        if offset > 0:
            leftVel =  midVel + offset
            rightVel = midVel
        else:
            leftVel = midVel
            rightVel = midVel - offset
    else:
        leftVel = offset
        rightVel = -offset

    if GetRC(5) > 50:   # GEAR switch = reverse
        leftVel = -leftVel
        rightVel = -rightVel        

    print(leftVel, rightVel)
    SetVel(0, leftVel, RCaccel)
    SetVel(1, rightVel, RCaccel)
    SetVel(2, leftVel, RCaccel)
    SetVel(3, rightVel, RCaccel)
