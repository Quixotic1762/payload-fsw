#!/usr/bin/env python3

"""
LoRa Interface for Raspberry Pi 5
---------------------------------
This script demonstrates how to interface a LoRa radio module (SX127x) with a Raspberry Pi 5.

Connections:
- GPIO4 (pin 7)   -> RESET
- GPIO17 (pin 11) -> DIO0
- GPIO10 (pin 19) -> MOSI
- GPIO9 (pin 21)  -> MISO
- GPIO11 (pin 23) -> SCK
- GPIO8 (pin 24)  -> NSS (CS)
- Optional: GPIO23, GPIO24, GPIO25 -> DIO1, DIO2, DIO3
"""

import time
import sys
import spidev
from RPi import GPIO

# Define the pins we're using
RESET_PIN = 4
DIO0_PIN = 17
DIO1_PIN = 23  # Optional
DIO2_PIN = 24  # Optional
DIO3_PIN = 25  # Optional
CS_PIN = 8     # SPI_CE0

# SX127X Register definitions
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0c
REG_FIFO_ADDR_PTR = 0x0d
REG_FIFO_TX_BASE_ADDR = 0x0e
REG_FIFO_RX_BASE_ADDR = 0x0f
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_SNR_VALUE = 0x19
REG_PKT_RSSI_VALUE = 0x1a
REG_MODEM_CONFIG_1 = 0x1d
REG_MODEM_CONFIG_2 = 0x1e
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_FREQ_ERROR_MSB = 0x28
REG_FREQ_ERROR_MID = 0x29
REG_FREQ_ERROR_LSB = 0x2a
REG_RSSI_WIDEBAND = 0x2c
REG_DETECTION_OPTIMIZE = 0x31
REG_INVERTIQ = 0x33
REG_DETECTION_THRESHOLD = 0x37
REG_SYNC_WORD = 0x39
REG_INVERTIQ2 = 0x3b
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42
REG_PA_DAC = 0x4d

# LoRa mode bits
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x05
MODE_RX_SINGLE = 0x06
MODE_CAD = 0x07

# IRQ flags
IRQ_TX_DONE_MASK = 0x08
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20
IRQ_RX_DONE_MASK = 0x40

# DIO mapping
DIO0_RX_DONE = 0x00
DIO0_TX_DONE = 0x40
DIO0_CAD_DONE = 0x80

class LoRa:
    def __init__(self, 
                 frequency=433.0, 
                 bandwidth=500000, 
                 coding_rate=5, 
                 spreading_factor=9, 
                 tx_power=17,
                 verbose=False):
        
        self.verbose = verbose
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup pins
        GPIO.setup(RESET_PIN, GPIO.OUT)
        GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        # Optional DIO pins
        GPIO.setup(DIO1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(DIO2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(DIO3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        # Setup SPI
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)  # Bus 0, Device 0 (CE0)
            self.spi.max_speed_hz = 5000000  # 5 MHz
            self.spi.mode = 0
        except Exception as e:
            print(f"Error initializing SPI: {e}")
            GPIO.cleanup()
            sys.exit(1)
        
        # Reset the module
        self.reset()
        
        # Check version
        version = self.read_register(REG_VERSION)
        if version != 0x12:
            raise Exception(f"LoRa initialization failed! Unexpected version: 0x{version:02x}")
        
        # Set to sleep mode
        self.sleep()
        
        # Set frequency
        self.set_frequency(frequency)
        
        # Set base addresses
        self.write_register(REG_FIFO_TX_BASE_ADDR, 0)
        self.write_register(REG_FIFO_RX_BASE_ADDR, 0)
        
        # Set LNA boost
        self.write_register(REG_LNA, self.read_register(REG_LNA) | 0x03)
        
        # Set auto AGC
        self.write_register(REG_MODEM_CONFIG_3, 0x04)
        
        # Set output power
        self.set_tx_power(tx_power,outputpin=1)
        
        # Configure modem
        self.set_bandwidth(bandwidth)
        self.set_coding_rate(coding_rate)
        self.set_spreading_factor(spreading_factor)
        #self.set_sync_word(0x12)  # Default sync word
        self.set_sync_word(0x34)
        self.set_preamble_length(12)
        
        # Set to standby mode
        self.idle()
        
        if self.verbose:
            print("LoRa initialization complete!")
    
    def reset(self):
        """Reset the LoRa module"""
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(0.1)
    
    def read_register(self, address):
        """Read a register from the LoRa module"""
        return self.spi.xfer([address & 0x7F, 0])[1]
    
    def write_register(self, address, value):
        """Write to a register in the LoRa module"""
        self.spi.xfer([address | 0x80, value])
    
    def sleep(self):
        """Put the module into sleep mode"""
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
    
    def idle(self):
        """Put the module into standby mode"""
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)
    
    def set_frequency(self, freq_mhz):
        """Set the frequency in MHz"""
        freq = int(freq_mhz * 16384.0)  # (freq * 2^19) / 32
        
        self.write_register(REG_FRF_MSB, (freq >> 16) & 0xFF)
        self.write_register(REG_FRF_MID, (freq >> 8) & 0xFF)
        self.write_register(REG_FRF_LSB, freq & 0xFF)
    
    def set_tx_power(self, level, outputPin=0):
        """Set transmit power level"""
        if outputPin == 0:  # RFO pin
            # Limit between 0 and 14 dBm
            level = max(0, min(14, level))
            self.write_register(REG_PA_CONFIG, 0x70 | level)
        else:  # PA_BOOST pin
            # Limit between 2 and 17 dBm
            level = max(2, min(17, level))
            if level > 17:
                # High power +20 dBm settings
                self.write_register(REG_PA_DAC, 0x87)
                self.write_register(REG_PA_CONFIG, 0x80 | (level - 5))
            else:
                self.write_register(REG_PA_DAC, 0x84)
                self.write_register(REG_PA_CONFIG, 0x80 | (level - 2))
    
    def set_bandwidth(self, bandwidth):
        """Set the bandwidth (in Hz)"""
        bw_value = 9  # Default 125kHz
        
        if bandwidth <= 7800:
            bw_value = 0
        elif bandwidth <= 10400:
            bw_value = 1
        elif bandwidth <= 15600:
            bw_value = 2
        elif bandwidth <= 20800:
            bw_value = 3
        elif bandwidth <= 31250:
            bw_value = 4
        elif bandwidth <= 41700:
            bw_value = 5
        elif bandwidth <= 62500:
            bw_value = 6
        elif bandwidth <= 125000:
            bw_value = 7
        elif bandwidth <= 250000:
            bw_value = 8
        
        # Read current value and update only bandwidth bits (first 4 bits)
        reg = self.read_register(REG_MODEM_CONFIG_1)
        self.write_register(REG_MODEM_CONFIG_1, (reg & 0x0F) | (bw_value << 4))
    
    def set_coding_rate(self, denominator):
        """Set error coding rate denominator (4 to 8)"""
        denominator = max(4, min(8, denominator))
        
        cr_value = denominator - 4
        
        reg = self.read_register(REG_MODEM_CONFIG_1)
        self.write_register(REG_MODEM_CONFIG_1, (reg & 0xF1) | (cr_value << 1))
    
    def set_spreading_factor(self, sf):
        """Set spreading factor (6 to 12)"""
        sf = max(6, min(12, sf))
        
        if sf == 6:
            self.write_register(REG_DETECTION_OPTIMIZE, 0xC5)
            self.write_register(REG_DETECTION_THRESHOLD, 0x0C)
        else:
            self.write_register(REG_DETECTION_OPTIMIZE, 0xC3)
            self.write_register(REG_DETECTION_THRESHOLD, 0x0A)
        
        reg = self.read_register(REG_MODEM_CONFIG_2)
        self.write_register(REG_MODEM_CONFIG_2, (reg & 0x0F) | (sf << 4))
    
    def set_sync_word(self, sw):
        """Set the sync word"""
        self.write_register(REG_SYNC_WORD, sw)
    
    def set_preamble_length(self, length):
        """Set preamble length"""
        self.write_register(REG_PREAMBLE_MSB, (length >> 8) & 0xFF)
        self.write_register(REG_PREAMBLE_LSB, length & 0xFF)
    
    def receive(self, callback=None, timeout=5000):
        """Set up module for reception"""
        # Enable RX interrupts
        self.write_register(REG_DIO_MAPPING_1, 0x00)  # DIO0 maps to RxDone
        
        # Clear IRQ flags
        self.write_register(REG_IRQ_FLAGS, 0xFF)
        
        # Put into continuous RX mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)
        
        start_time = time.time()
        
        while True:
            if GPIO.input(DIO0_PIN):
                # Read IRQ flags
                irq_flags = self.read_register(REG_IRQ_FLAGS)
                
                # Clear IRQ flags
                self.write_register(REG_IRQ_FLAGS, 0xFF)
                
                if (irq_flags & IRQ_RX_DONE_MASK) and not (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK):
                    # Get packet length
                    packet_length = self.read_register(REG_RX_NB_BYTES)
                    
                    # Set FIFO pointer
                    self.write_register(REG_FIFO_ADDR_PTR, self.read_register(REG_FIFO_RX_CURRENT_ADDR))
                    
                    # Read payload
                    payload = []
                    for _ in range(packet_length):
                        payload.append(self.read_register(REG_FIFO))
                    
                    # Calculate RSSI
                    rssi = self.read_register(REG_PKT_RSSI_VALUE) - 157
                    
                    # Call callback if provided
                    if callback:
                        callback(bytes(payload), rssi)
                    
                    return bytes(payload), rssi
            
            # Check for timeout
            if timeout and (time.time() - start_time) * 1000 > timeout:
                return None, None
            
            time.sleep(0.01)  # Small delay to avoid CPU hammering
    
    def send(self, data, wait=True, timeout=5000):
        """Send a packet of data"""
        # Set to standby mode
        self.idle()
        
        # Set DIO0 to map to TxDone
        self.write_register(REG_DIO_MAPPING_1, DIO0_TX_DONE)
        
        # Clear IRQ flags
        self.write_register(REG_IRQ_FLAGS, 0xFF)
        
        # Set FIFO pointer
        self.write_register(REG_FIFO_ADDR_PTR, 0)
        
        # Write data to FIFO
        for byte in data:
            self.write_register(REG_FIFO, byte)
        
        # Set payload length
        self.write_register(REG_PAYLOAD_LENGTH, len(data))
        
        # Start transmission
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)
        
        # Wait for transmission to complete if requested
        if wait:
            start_time = time.time()
            
            while True:
                if GPIO.input(DIO0_PIN):
                    # Clear IRQ flags
                    self.write_register(REG_IRQ_FLAGS, 0xFF)
                    
                    # Return to standby mode
                    self.idle()
                    return True
                
                # Check for timeout
                if timeout and (time.time() - start_time) * 1000 > timeout:
                    return False
                
                time.sleep(0.01)  # Small delay to avoid CPU hammering
        
        return True
    
    def close(self):
        """Clean up"""
        if hasattr(self, 'spi'):
            self.spi.close()
        GPIO.cleanup()


# Example usage
if __name__ == "__main__":
    try:
        print("Initializing LoRa module...")
        lora = LoRa(frequency=433.0, bandwidth=125000, tx_power=17, verbose=True)
        
        # Example: Send a message
        message = "Hello from Raspberry Pi 5!"
        print(f"Sending message: {message}")
        lora.send(message.encode())
        print("Message sent!")
        
        # Example: Receive messages for 30 seconds
        print("Listening for messages (30 seconds)...")
        
        def on_receive(payload, rssi):
            print(f"Received message: {payload.decode('utf-8', errors='replace')}")
            print(f"RSSI: {rssi} dBm")
        
        start_time = time.time()
        while time.time() - start_time < 30:
            lora.receive(callback=on_receive, timeout=1000)
        
        print("Done listening.")
        lora.close()
        
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
        if 'lora' in locals():
            lora.close()
    except Exception as e:
        print(f"Error: {e}")
        if 'lora' in locals():
            lora.close()

