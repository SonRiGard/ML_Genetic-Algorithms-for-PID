    
# Importing Libraries
# import serial
# import time

# ACK_start_GA = 111.1 

# kp = 1.1
# ki = 1.2
# kd = 1.3

# ser = serial.Serial(port='COM9', baudrate=9600, timeout=0.1)

# def write_read(kp,ki,kd):
#     # ser.write(bytes(str(kp), 'utf-8'))
#     # ser.write(bytes(str(ki), 'utf-8'))
#     ser.write(bytes(str(kd), 'utf-8'))
#     time.sleep(0.08)
#     data = ser.readline()
#     return data

# def sent_ACK_start (ACK):
#     ser.write(bytes(str(ACK), 'utf-8'))
#     time.sleep(1)
#     data = ser.readline()
#     floatdata = 0
#     if is_float(data) == True:
#         floatdata = float(data)   
#         if floatdata == 222.2:
#             return 1
#         else :
#             return  0

# def is_float(element: any) -> bool:
#     #If you expect None to be passed:
#     if element is None: 
#         return False
#     try:
#         float(element)
#         return True
#     except ValueError:
#         return False


# while(sent_ACK_start(ACK_start_GA) == 0):
#     print("Wait ACk confirm")
  
# while True:
#     # num = input("Enter a number: ") # Taking input from user
#     value = write_read(kp,ki,kd)
#     print(value) # printing the value


# Importing Libraries
import serial
import time
ser = serial.Serial(port='COM5', baudrate=115200, timeout=.1)
def write_read(x):
    ser.write(bytes(x, 'utf-8'))
    time.sleep(0.10)
    data = ser.readline()
    print(data)
    return data

def sent_Kpid (ukp,uki,ukd):
    print("seding kp : ...")
    num = input("Enter a number: ") # Taking input from user
    ser.write(bytes(str(ukp), 'utf-8'))
    time.sleep(0.011)
    print("sent kp!")
    
    print("seding kI : ...")
    num = input("Enter a number: ") # Taking input from user
    ser.write(bytes(str(uki), 'utf-8'))
    time.sleep(0.011)
    print("sent ki!")
        
    print("seding kd : ...")
    num = input("Enter a number: ") # Taking input from user
    ser.write(bytes(str(ukd), 'utf-8'))
    time.sleep(0.011)
    print("sent kp!")
    
    while (True):
        if(ser.in_waiting != 0):
            break 
        
    # ACK_start_cur_pid =  ser.readline().decode().rstrip()
    ACK_start_cur_pid=""
    while (ACK_start_cur_pid != "111"):
        ACK_start_cur_pid =  ser.readline().decode().rstrip()
    print(ACK_start_cur_pid)
    return True
    
    
def process_simulat ():
    Rx_data=ser.readline()
    print(Rx_data)
    # print(data[0]+data[1]+data[2])
    data = Rx_data.decode().rstrip().split(',')

    print(data)
    if len(data) == 3:
        # print(data[0]+data[1]+data[2])
        # # Data for a three-dimensional line
        print(data[0])
        print("    ")
        print(data[1])
        print("\n")
        
        if data[2] == "222":#ACk stop process
            return 0#end of 
        else:
            return 1#continued receiver in main function
    time.sleep(0.0025)
kp = 1
ki = 1
kd = 1


while True:
    # value = write_read("1.1")
    # value = write_read("2.1")
    # value = write_read("4.1")
    # print(value) # printing the valued

    kp += 1
    ki += 1
    kd += 1
    sent_Kpid(kp,ki,kd)
    while (1):
        temp = ser.readline()
        if (process_simulat() == 0):
            break
           
    # time.sleep(0.001)
    # data = arduino.readline()
    # print(data)
    # time.sleep(0.01)

