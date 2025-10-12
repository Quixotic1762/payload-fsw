def gsm_proc(sms_payload):
    import serial
    import time

    number = '+919699060432'
    ser = serial.Serial("/dev/ttyAMA1", 9600, timeout=1)

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
    time.sleep(5)
    while True:
        
        send_sms(number, sms_payload)
        time.sleep(2)

if __name__ == "__main__":
    gsm_proc("#,050,343,308,29.00,941.00,10.00,2139.0,-60.83,-0.04,-0.00,0.98,0.24,0.31,0.37,74.15,-72.38,-109.08,1,18.48,73.82,569.539,140216.000,34,3.68,1.78,54.3,0,0,00,1,$")