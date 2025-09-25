def lora(telm_q, tx_enable):
    """    
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
    from datetime import datetime

    # Import the LoRa class from our main file
    # Make sure lora_rpi5_interface.py is in the same directory
    try:
        from lora_rpi5_interface import LoRa
    except ImportError as e:
        print(f"Import error: {e}")
        sys.exit(1)

    def main():
        try:
            lora = LoRa(
                frequency=433.0,
                bandwidth=500000,
                spreading_factor=12,
                coding_rate=5,
                tx_power=17,
                verbose=False
            )
            #lora = LoRa()
            while True:
                if (telm_q.qsize() > 0 and (tx_enable.is_set())):
                    message = telm_q.get()
                    lora.send(message.encode())
                    time.sleep(1)                     
                payload, rssi = lora.receive(timeout=1000)
                if payload:
                    print(payload.decode('utf-8'))
                    if payload.decode('utf-8') == 'ack':
                        tx_enable.set()
    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if 'lora' in locals():
                lora.close()
                print("LoRa resources released.")
    main()