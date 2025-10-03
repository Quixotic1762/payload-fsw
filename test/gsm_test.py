def gsm_proc(sms_payload):
    import serial
    import time

    number = '+918984955170'
    ser = serial.Serial("/dev/ttyAMA1", 9600, timeout=1)

    def send_at(command):
        #tr_buffer = (command+'\r\n').encode(encoding="utf-8")
        ser.write((command+'\r\n').encode())
        while ser.readline():
            print(f"{command}")
            print(ser.readline().decode('utf-8'))

    def send_sms(number, message):
        send_at("AT+CMGF=1")
        set_nu = 'AT+CMGS="'+number+'"'
        send_at(set_nu)
        msg = message + chr(26)
        msg_bytes = msg.encode()
        ser.write(msg_bytes)

    #send_at("AT+IPR=115200")

    while True:
        sms_payload = "#,ASI-ROCKETRY-050,2,0,29.00,1013.25,779.0,-67.21,-1.58,-1.05,-3.79,903.15,12.906797,77.595902,12,1.5843,-32,1,1502077652,$"
        send_sms(number, sms_payload)
        time.sleep(1)

gsm_proc("test")