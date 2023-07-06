    
# Importing Libraries
import serial
import time

ACK_start_GA = 111.1 

kp = 1.1
ki = 1.2
kd = 1.3

ser = serial.Serial(port='COM9', baudrate=9600, timeout=0.1)

def write_read(kp,ki,kd):
    # ser.write(bytes(str(kp), 'utf-8'))
    # ser.write(bytes(str(ki), 'utf-8'))
    ser.write(bytes(str(kd), 'utf-8'))
    time.sleep(0.08)
    data = ser.readline()
    return data

def sent_ACK_start (ACK):
    ser.write(bytes(str(ACK), 'utf-8'))
    time.sleep(1)
    data = ser.readline()
    floatdata = 0
    if is_float(data) == True:
        floatdata = float(data)   
        if floatdata == 222.2:
            return 1
        else :
            return  0

def is_float(element: any) -> bool:
    #If you expect None to be passed:
    if element is None: 
        return False
    try:
        float(element)
        return True
    except ValueError:
        return False


while(sent_ACK_start(ACK_start_GA) == 0):
    print("Wait ACk confirm")
  
while True:
    # num = input("Enter a number: ") # Taking input from user
    value = write_read(kp,ki,kd)
    print(value) # printing the value


