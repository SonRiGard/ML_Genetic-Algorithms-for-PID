    
# Importing Libraries
import serial
import time



ser = serial.Serial(port='COM3', baudrate=115200, timeout=1)

def sent_Kpid (ukp,uki,ukd):
    time.sleep(2)
    print("seding kp : ...")
    ser.write(bytes(str(ukp)+"\n", 'utf-8'))
    time.sleep(0.1)
    print("sent kp!")
    
    print("seding kI : ...")
    ser.write(bytes(str(uki)+"\n", 'utf-8'))
    time.sleep(0.1)
    print("sent ki!")
        
    print("seding kd : ...")
    ser.write(bytes(str(ukd)+"\n", 'utf-8'))
    print("sent kp!")
    time.sleep(0.1)

    ACK_start_cur_pid=""
    while (ACK_start_cur_pid != "111"):
        ACK_start_cur_pid =  ser.readline().decode().rstrip()
    print(ACK_start_cur_pid)
    return True
    
    
def process_simulat (N_time_out):
    Time_out = 0
    # ser.reset_input_buffer()
    Rx_data=ser.readline()
    if Rx_data == '':
        Time_out += 1
    print(Rx_data)
    # print(data[0]+data[1]+data[2])
    data = Rx_data.decode().rstrip().split(',')
    print(data[0])  
   
    if Time_out < N_time_out :
        if len(data) == 3:
            if data[2] == "222":#ACk stop process
                return 0#end of 
            else:
                return 1#continued receiver in main function
    else :
        return 0
kp = 1
ki = 1
kd = 1


while True:
    kp += 1
    ki += 1
    kd += 1
    sent_Kpid(kp,ki,kd)
    while (1):
        # temp = ser.readline()
        if (process_simulat(5) == 0):
            break

           
    # time.sleep(0.001)
    # data = arduino.readline()
    # print(data)
    # time.sleep(0.01)

