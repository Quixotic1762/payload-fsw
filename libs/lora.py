def lora():
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
        print("Make sure lora_rpi5_interface.py is in the same directory as this script")
        sys.exit(1)

    def send_mode(lora, count=100):
        """Send test messages repeatedly"""
        counter = 0
        try:
            #while counter < count:
            while True:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                line = subprocess.check_output(['tail','-1',"/home/pi5/Documents/rssi_log.csv"])
                blenano_str = ','.join(nano_q.get()) # convert ble list to string
                gnss_str = ','.join(gnss_q.get()) # convert gnss list to string
                message = str(line)+ str(blenano_str)+str(counter)+str(gnss_str)
                ## generate_telemetry() 

                #message = f"RPi5 LoRa Test #{counter} at {timestamp}"
                print(f"Sending: {message}")

                # Send the message
                lora.send(message.encode())
                print("Message sent!")

                counter += 1
                print(f"Waiting 5 seconds... ({counter}/{count})")
                time.sleep(0.3)
        except KeyboardInterrupt:
            print("\nSending stopped by user.")
        finally:
            print(f"Sent {counter} messages.")

    def receive_mode(lora, timeout=None):
        """Listen for incoming messages"""
        message_count = 0
        start_time = time.time()

        try:
            print("Listening for LoRa messages... Press Ctrl+C to stop.")

            while True:
                if timeout and (time.time() - start_time > timeout):
                    print(f"Receive timeout after {timeout} seconds.")
                    break

                # Try to receive a packet (1 sec timeout per attempt)
                payload, rssi = lora.receive(timeout=1000)

                if payload:
                    message_count += 1
                    try:
                        decoded = payload.decode('utf-8')
                        print(f"\nMessage #{message_count} received:")
                        print(f"Data: {decoded}")
                        print(f"RSSI: {rssi} dBm")
                        print(f"Length: {len(payload)} bytes")
                    except UnicodeDecodeError:
                        print(f"\nBinary message received:")
                        print(f"Data (hex): {payload.hex()}")
                        print(f"RSSI: {rssi} dBm")
                        print(f"Length: {len(payload)} bytes")
                else:
                    # Print a dot to show we're still alive
                    sys.stdout.write(".")
                    sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nReceiving stopped by user.")
        finally:
            print(f"Received {message_count} messages.")

    def main():
        parser = argparse.ArgumentParser(description='Test LoRa functionality')
        parser.add_argument('--mode', choices=['send', 'recv'], required=True, 
                            help='Operation mode: send or receive')
        parser.add_argument('--freq', type=float, default=433.0,
                            help='Frequency in MHz (default: 433.0)')
        parser.add_argument('--bw', type=int, default=500000,
                            help='Bandwidth in Hz (default: 125000)')
        parser.add_argument('--sf', type=int, default=12,
                            help='Spreading Factor (default: 9)')
        parser.add_argument('--cr', type=int, default=5,
                            help='Coding Rate denominator (default: 5, which is 4/5)')
        parser.add_argument('--power', type=int, default=18,
                            help='TX Power in dBm (default: 17)')
        parser.add_argument('--count', type=int, default=100,
                            help='Number of messages to send (default: 100)')
        parser.add_argument('--timeout', type=int, default=None,
                            help='Timeout in seconds for receive mode (default: none/infinite)')
        parser.add_argument('--debug', action='store_true',
                            help='Enable debug mode with more verbose output')
        
        args = parser.parse_args()
        
        print("LoRa Test Application")
        print("=====================")
        print(f"Mode: {args.mode}")
        print(f"Frequency: {args.freq} MHz")
        print(f"Bandwidth: {args.bw} Hz")
        print(f"Spreading Factor: {args.sf}")
        print(f"Coding Rate: 4/{args.cr}")
        print(f"TX Power: {args.power} dBm")
        
        try:
            # Initialize LoRa
            print("\nInitializing LoRa module...")
            
            # Check SPI functionality first
            if args.debug:
                try:
                    import spidev
                    spi = spidev.SpiDev()
                    spi.open(0, 0)
                    print("SPI initialized successfully")
                    spi.close()
                except Exception as e:
                    print(f"SPI initialization failed: {e}")
                    print("Please check if SPI is enabled in raspi-config")
                    sys.exit(1)
            
            # Initialize LoRa
            lora = LoRa(
                frequency=args.freq,
                bandwidth=args.bw,
                spreading_factor=args.sf,
                coding_rate=args.cr,
                tx_power=args.power,
                verbose=args.debug
            )
            print("LoRa initialized successfully!")
            
            # Run selected mode
            if args.mode == 'send':
                send_mode(lora, args.count)
            else:  # 'recv'
                receive_mode(lora, args.timeout)
                
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if 'lora' in locals():
                lora.close()
                print("LoRa resources released.")
    
    #if __name__ == "__main__":
    main()