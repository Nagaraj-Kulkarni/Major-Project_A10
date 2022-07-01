#!/usr/bin/env python3

# library imports
import serial
import time
import RPi.GPIO as GPIO
import smbus

'''
Accident Detectect robo
'''

### Global variables

# UART ports
GPS_PORT = "/dev/ttyAMA0"
GSM_PORT = "/dev/ttyAMA1"

# SMS receipient
SMS_NUM = "+919449189952"

# GPIO Pins

# Buzzer
GPIO_BUZZER=12

# ultrasound front
GPIO_TRIGG_FRONT=19
GPIO_ECHO_FRONT=26

# ultrasound back
GPIO_TRIGG_BACK=20
GPIO_ECHO_BACK=21

# GRYO
GRYO_ADDRESS = 0x68   # MPU6050 device address

##################################

### app variables
GPS_LAT=""
GPS_LONG=""

# measurements from ultrasound sensor
DIST_FRONT=-1
DIST_BACK=-1

# gryo angle
GYRO_X=0
GYRO_Y=0
GYRO_Z=0

# ACC angle
ACC_X=0
ACC_Y=0
ACC_Z=0

IS_SMS_SENT=False

# GPIO Setup
def setupGPIO():
    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    # setup Buzzer
    global GPIO_BUZZER
    GPIO.setup(GPIO_BUZZER, GPIO.OUT)
    GPIO.output(GPIO_BUZZER, False)

    # setup ultrasound
    global GPIO_TRIGG_FRONT
    global GPIO_ECHO_FRONT
    GPIO.setup(GPIO_TRIGG_FRONT, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
    GPIO.output(GPIO_TRIGG_FRONT, False)

    global GPIO_TRIGG_BACK
    global GPIO_ECHO_BACK
    GPIO.setup(GPIO_TRIGG_BACK, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_BACK, GPIO.IN)
    GPIO.output(GPIO_TRIGG_BACK, False)

# smbus.SMBus(0) for older version boards
SMBUS = smbus.SMBus(1)

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38

ACCEL_XOUT_H = 0x43
ACCEL_YOUT_H = 0x45
ACCEL_ZOUT_H = 0x47

GYRO_XOUT_H  = 0x3B
GYRO_YOUT_H  = 0x3D
GYRO_ZOUT_H  = 0x3F

# setup I2c Gyro
def MPU_Init():
        #write to sample rate register
        SMBUS.write_byte_data(GRYO_ADDRESS, SMPLRT_DIV, 7)

        #Write to power management register
        SMBUS.write_byte_data(GRYO_ADDRESS, PWR_MGMT_1, 1)

        #Write to Configuration register
        SMBUS.write_byte_data(GRYO_ADDRESS, CONFIG, 0)

        #Write to Gyro configuration register
        SMBUS.write_byte_data(GRYO_ADDRESS, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        SMBUS.write_byte_data(GRYO_ADDRESS, INT_ENABLE, 1)

# read from gryo
def read_raw_gyro(addr):
        #Accelero and Gyro value are 16-bit
    high = SMBUS.read_byte_data(GRYO_ADDRESS, addr)
    low = SMBUS.read_byte_data(GRYO_ADDRESS, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value


# Send SMS using UART
def sendSMS(message):
    global GSM_PORT

    GSM_SERIAL = serial.Serial(GSM_PORT,  9600, timeout=5)
    try:
        time.sleep(0.5)
        GSM_SERIAL.write(b'ATZ\r')
        time.sleep(0.5)
        GSM_SERIAL.write(b'AT+CMGF=1\r')
        time.sleep(0.5)
        GSM_SERIAL.write(b'AT+CMGS="' + SMS_NUM.encode() + b'"\r')
        time.sleep(0.5)
        GSM_SERIAL.write(message.encode() + b"\r")
        time.sleep(0.5)
        GSM_SERIAL.write(bytes([26]))
        time.sleep(0.5)
    except:
        print ("[ERROR] error communicating with GSM")
        GSM_SERIAL.close()
    finally:
        GSM_SERIAL.close()

# Read GPS from UART
def readGPS():
    global GPS_PORT
    global GPS_LAT
    global GPS_LONG

    # GPS_SERIAL = serial.Serial(GPS_PORT, 9600, timeout=1)
    # GPS_SERIAL.reset_input_buffer()
    gpsValue = open("/home/pi/gps.txt", "r") 
    try:
        rawData = gpsValue.readline()
        data = rawData.split(",")
        GPS_LAT = data[0]
        GPS_LONG = data[1]
        # parseGPS()
    except:
        print ("[ERROR] error reading GPS")
        # GPS_SERIAL.close()
        gpsValue.close()
    finally:
        # GPS_SERIAL.close()
        gpsValue.close()

# read Distance
def readDist(trigg_pin, echo_pin):
    # set Trigger to HIGH
    GPIO.output(trigg_pin, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trigg_pin, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(echo_pin) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(echo_pin) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    return (TimeElapsed * 34300) / 2

def notifyBuzzer():
    global GPIO_BUZZER
    GPIO.output(GPIO_BUZZER, True)
    time.sleep(.25)
    GPIO.output(GPIO_BUZZER, False)
    time.sleep(.25)


def readDistFront():
    global GPIO_TRIGG_FRONT
    global GPIO_ECHO_FRONT
    global DIST_FRONT

    print("Reading Front")
    DIST_FRONT = readDist(GPIO_TRIGG_FRONT, GPIO_ECHO_FRONT)

def readDistBack():
    global GPIO_TRIGG_BACK
    global GPIO_ECHO_BACK
    global DIST_BACK

    print("Reading Back")
    DIST_BACK = readDist(GPIO_TRIGG_BACK, GPIO_ECHO_BACK)

# read gryo
def readGyro():
    global GYRO_X
    global GYRO_Y
    global GYRO_Z

    global ACC_X
    global ACC_Y
    global ACC_Z

    # print("Reading Gyro")

    #Read Accelerometer raw value
    AccX = read_raw_gyro(ACCEL_XOUT_H)
    AccY = read_raw_gyro(ACCEL_YOUT_H)
    AccZ = read_raw_gyro(ACCEL_ZOUT_H)

    ACC_X = AccX/131.0
    ACC_Y = AccY/131.0
    ACC_Z = AccZ/131.0

    #Read Gyroscope raw value
    GyroX = read_raw_gyro(GYRO_XOUT_H)
    GyroY = read_raw_gyro(GYRO_YOUT_H)
    GyroZ = read_raw_gyro(GYRO_ZOUT_H)

    #Full scale range +/- 250 degree/C as per sensitivity scale factor

    # X
    GYRO_X = (GyroX/16384.0)

    # Y
    GYRO_Y = -(GyroY/16384.0)

    # Z
    GYRO_Z = -(GyroZ/16384.0)


def notifySMS():
    global IS_SMS_SENT
    global GPS_LAT
    global GPS_LONG

    # print("SMS " + IS_SMS_SENT)
    if IS_SMS_SENT == True :
        print("EMERGENCY NOTIFIED!")
    else:
        readGPS()
        message = "Accident OCCURED!\n\nhttp://maps.google.com/maps?q=loc:%s" % GPS_LAT + ",%s" % GPS_LONG
        IS_SMS_SENT = True
        print("SMS SENT!")
        sendSMS(message)
        # print(message)

# process data from sensors and notify
def processData():
    global GYRO_X
    global GYRO_Y
    global GYRO_Z

    global ACC_X
    global ACC_Y
    global ACC_Z

    global DIST_FRONT
    global DIST_BACK

    print("x: %.2f" %GYRO_X, "\ty: %.2f" %GYRO_Y, "\tz: %.2f" %GYRO_Z, "\tfront: %.2fcm" % DIST_FRONT, "\tback: %.2fcm" % DIST_BACK)

    # # ACCident Zone
    if (GYRO_X > .7 or GYRO_X < -.7) or (GYRO_Y > .7 or GYRO_Y < -.7) or (GYRO_X < 0 and GYRO_Y < 0) or (GYRO_Z < -0.1):
        notifyBuzzer()
        if(GYRO_Z < 0 and GYRO_Y < 0):
            print("ACCIDENT!!")
            notifySMS()
        else:
            print("DANGER!!")

    elif ((abs(GYRO_X) > .5) and (abs(GYRO_X) < .7)) or ((abs(GYRO_Y) > .5) and (abs(GYRO_Y) < .7)):
        print("WARNING!")
    else:
        print("Good")


    ## Safe Distance
    if (DIST_FRONT < 10) or (DIST_BACK < 10):
        notifyBuzzer()
        print("WARNING: VECHICAL TOO CLOSE")


# Log Data to file
def logData():
    global GPS_LAT
    global GPS_LONG

    global GYRO_X
    global GYRO_Y
    global GYRO_Z

    global DIST_FRONT
    global DIST_BACK

    # open file stream
    dataFile = open("/home/pi/log.csv", "a") 

    try:
        # format    x,y,z,dist_front,dist_back,lat,long
        logdata = "%.2f" % GYRO_X + ",%.2f" % GYRO_Y + ",%.2f" % GYRO_Z + ",%.2f" % DIST_FRONT + ",%.2f" % DIST_BACK + ",%s" % GPS_LAT + ",%s" %GPS_LONG
        dataFile.write(logdata + "\n")
    except:
        print ("[ERROR] error while logging data")
        dataFile.close()
    finally:
        dataFile.close()


# MAIN function
if __name__ == '__main__':
    setupGPIO()

    # read GPS intially
    readGPS()

    try:
        # print("Reading Serial\n")
        while True:
            print("")

            # read distance
            readDistFront()
            readDistBack()

            # read Gyro
            readGyro()

            # process data
            processData()

            # log data
            logData()

            # delay before repeating
            time.sleep(0.5)
            # time.sleep(1)


    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Interrupted by user")
    except:
        GPIO.cleanup()
        print ("Application error!")
    finally:
        GPIO.cleanup()
 
