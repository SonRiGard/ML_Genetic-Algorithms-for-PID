1. First send data include Kp,ki,kd to Microcontrollers
2. Wait from microcontrollers ACK  = "111" to confirm receivered k pid:
    while (ACK_start_cur_pid != "111"):
