def lora(telm_q, tx_enable):
    #!/usr/bin/env python3

    """
    Author: Aman Gupta, Darsh Salian
    LoRa Test Script for Raspberry Pi 5
    -----------------------------------
    This script tests basic functionality of the LoRa module.
    It supports both sending and receiving modes.

    Usage:
      python lora_test.py --mode send   # Send test messages every 5 seconds
      python lora_test.py --mode recv   # Continuously listen for messages
    
    Connections:
    - GPIO4 (pin 7)   -> RESET
    - GPIO17 (pin 11) -> DIO0
    - GPIO10 (pin 19) -> MOSI
    - GPIO9 (pin 21)  -> MISO
    - GPIO11 (pin 23) -> SCK
    - GPIO8 (pin 24)  -> NSS (CS)
    - Optional: GPIO23, GPIO24, GPIO25 -> DIO1, DIO2, DIO3
    """
    import sys
    import time
    import argparse
    from datetime import datetime
    import subprocess

    # Import the LoRa class from our main file
    # Make sure lora_rpi5_interface.py is in the same directory
    try:
        from lora_rpi5_interface import LoRa
    except ImportError as e:
        print(f"Import error: {e}")
        sys.exit(1)

    '''
    sends telemetry recieved via queue telm_q
    send_mode(lora, telm_str): sends telm_str
    '''
    def send(lora):
        ''' send msg -> if retval = 1 -> send again'''
        message = telm_q.get()
        return lora.send(message.encode())

    def main():
        try:
            # Initialize LoRa            
            # Check SPI functionality first   
            # Initialize LoRa
            '''
            lora = LoRa(
                frequency=args.freq,
                bandwidth=args.bw,
                spreading_factor=args.sf,
                coding_rate=args.cr,
                tx_power=args.power,
                verbose=args.debug
            )
            '''
            lora = LoRa()
            while True:
                if (telm_q.qsize() > 0 and (tx_enable.value == 1)):
                    send(lora)
                        
                payload, rssi = lora.receive(timeout=1000)
                if payload:
                    if payload == 'ack':
                        tx_enable.value = 1
    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if 'lora' in locals():
                lora.close()
                # print("LoRa resources released.")
    
    #if __name__ == "__main__":
    main()