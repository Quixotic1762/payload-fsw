def gsm_proc(sms_payload):
    import serial
    import time

    number = '+918984955170'
    ser = serial.Serial("/dev/ttyAMA2", 9600, timeout=1)

    def send_at(command):
        #tr_buffer = (command+'\r\n').encode(encoding="utf-8")
        ser.write((command+'\r\n').encode())
        while ser.readline():
            print(ser.readline().decode('utf-8'))

    def send_sms(number, message):
        send_at("AT+CMGF=1")
        set_nu = 'AT+CMGS="'+number+'"'
        send_at(set_nu)
        msg = message + chr(26)
        msg_bytes = msg.encode()
        ser.write(msg_bytes)

    while True:
        send_sms(number, sms_payload)
        time.sleep(5)