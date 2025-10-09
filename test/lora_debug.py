#!/usr/bin/env python3
"""
LoRa Debug Script for Raspberry Pi 5
-----------------------------------
This script provides detailed debugging and testing for LoRa modules (SX127x) on Raspberry Pi 5.
It performs hardware tests, register dumps, and connection verification.

Usage:
  python lora_debug.py --mode all              # Run all tests
  python lora_debug.py --mode hardware         # Test hardware connections
  python lora_debug.py --mode registers        # Dump register values
  python lora_debug.py --mode ping             # Send a ping and wait for response
  python lora_debug.py --mode sweep            # Sweep through frequencies
  python lora_debug.py --mode scan             # Scan for LoRa activity

Connections:
- GPIO4 (pin 7)   -> RESET x -> 25
- GPIO17 (pin 11) -> DIO0 x -> 7
- GPIO10 (pin 19) -> MOSI -
- GPIO9 (pin 21)  -> MISO -
- GPIO11 (pin 23) -> SCK -
- GPIO8 (pin 24)  -> NSS (CS) -
- Optional: GPIO23, GPIO24, GPIO25 -> DIO1, DIO2, DIO3
"""

import time
import sys
import argparse
import json
import signal
from datetime import datetime

# Import the LoRa class from the interface file
try:
    from lora_rpi5_interface import LoRa
    import spidev
    from RPi import GPIO
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all required libraries are installed:")
    print("  pip install spidev RPi.GPIO")
    print("And make sure lora_rpi5_interface.py is in the same directory as this script")
    sys.exit(1)

# Define constants
#RESET_PIN = 4\
RESET_PIN = 25
#DIO0_PIN = 17
DIO0_PIN = 22
DIO1_PIN = 23
DIO2_PIN = 24
#DIO3_PIN = 25
CS_PIN = 8

# Define register names for better readability
REG_NAMES = {
    0x01: "REG_OP_MODE",
    0x06: "REG_FRF_MSB",
    0x07: "REG_FRF_MID",
    0x08: "REG_FRF_LSB",
    0x09: "REG_PA_CONFIG",
    0x0c: "REG_LNA",
    0x12: "REG_IRQ_FLAGS",
    0x13: "REG_RX_NB_BYTES",
    0x19: "REG_PKT_SNR_VALUE",
    0x1a: "REG_PKT_RSSI_VALUE",
    0x1d: "REG_MODEM_CONFIG_1",
    0x1e: "REG_MODEM_CONFIG_2",
    0x20: "REG_PREAMBLE_MSB",
    0x21: "REG_PREAMBLE_LSB",
    0x22: "REG_PAYLOAD_LENGTH",
    0x26: "REG_MODEM_CONFIG_3",
    0x39: "REG_SYNC_WORD",
    0x40: "REG_DIO_MAPPING_1",
    0x41: "REG_DIO_MAPPING_2",
    0x42: "REG_VERSION",
    0x44: "REG_TEMP",
    0x4d: "REG_PA_DAC",
}

# Define some useful constants for LoRa
REG_VERSION = 0x42
REG_OP_MODE = 0x01
REG_IRQ_FLAGS = 0x12

class LoRaDebug:
    def __init__(self, args):
        self.args = args
        self.lora = None
        self.spi = None
        self.debug_log = []
        self.setup_signal_handler()
        
    def setup_signal_handler(self):
        """Set up a signal handler for clean exit"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        """Handle SIGINT and SIGTERM signals"""
        print("\nReceived termination signal. Cleaning up...")
        self.cleanup()
        sys.exit(0)
        
    def log(self, message, level="INFO"):
        """Log a message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_entry = f"{timestamp} [{level}] {message}"
        print(log_entry)
        self.debug_log.append(log_entry)
        
    def save_log(self, filename="lora_debug.log"):
        """Save debug log to a file"""
        with open(filename, 'w') as f:
            for entry in self.debug_log:
                f.write(f"{entry}\n")
        self.log(f"Debug log saved to {filename}")
        
    def test_hardware(self):
        """Test hardware connections"""
        self.log("Testing hardware connections...", "TEST")
        
        # Test GPIO
        self.log("Testing GPIO setup...")
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Test RESET pin
            GPIO.setup(RESET_PIN, GPIO.OUT)
            GPIO.output(RESET_PIN, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(RESET_PIN, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(RESET_PIN, GPIO.HIGH)
            time.sleep(0.1)
            self.log("RESET pin (GPIO4) test successful")
            
            # Test DIO pins
            GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(DIO1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(DIO2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            #GPIO.setup(DIO3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            self.log("No Problem with DIO setup")
            
            dio0_state = GPIO.input(DIO0_PIN)
            dio1_state = GPIO.input(DIO1_PIN)
            dio2_state = GPIO.input(DIO2_PIN)
            #dio3_state = GPIO.input(DIO3_PIN)
            self.log("No Problem with DIO input")

            
            self.log(f"DIO0 pin (GPIO7) state: {dio0_state}")
            self.log(f"DIO1 pin (GPIO23) state: {dio1_state}")
            self.log(f"DIO2 pin (GPIO24) state: {dio2_state}")
            #self.log(f"DIO3 pin (GPIO25) state: {dio3_state}")
            self.log("No Problem with DIO state..")

            
            GPIO.cleanup()
            self.log("GPIO test completed")
            
        except Exception as e:
            self.log(f"GPIO test failed: {e}", "ERROR")
            GPIO.cleanup()
            return False
            
        # Test SPI
        self.log("Testing SPI connection...")
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)  # Bus 0, Device 0 (CE0)
            self.spi.max_speed_hz = 5000000  # 5 MHz
            self.spi.mode = 0
            self.log("SPI connection established")
            self.spi.close()
        except Exception as e:
            self.log(f"SPI test failed: {e}", "ERROR")
            self.log("Make sure SPI is enabled in raspi-config", "ERROR")
            return False
            
        # Test LoRa module
        self.log("Testing LoRa module presence...")
        try:
            self.lora = LoRa(
                frequency=self.args.freq,
                bandwidth=self.args.bw,
                spreading_factor=self.args.sf,
                coding_rate=self.args.cr,
                tx_power=self.args.power,
                verbose=False
            )
            
            # Read version register
            version = self.lora.read_register(REG_VERSION)
            self.log(f"LoRa chip version: 0x{version:02x}")
            
            if version == 0x12:
                self.log("LoRa module SX1276/77/78/79 detected ✓", "SUCCESS")
            else:
                self.log(f"Unexpected chip version: 0x{version:02x}. Expected 0x12 for SX127x", "WARNING")
                
            self.lora.close()
            self.lora = None
            return True
            
        except Exception as e:
            self.log(f"LoRa module test failed: {e}", "ERROR")
            if self.lora:
                self.lora.close()
                self.lora = None
            return False
            
    def dump_registers(self):
        """Dump all important registers"""
        self.log("Dumping LoRa registers...", "TEST")
        
        try:
            self.lora = LoRa(
                frequency=self.args.freq,
                bandwidth=self.args.bw,
                spreading_factor=self.args.sf,
                coding_rate=self.args.cr,
                tx_power=self.args.power,
                verbose=False
            )
            
            # Read all important registers
            registers = {}
            for addr in range(0x01, 0x50):
                value = self.lora.read_register(addr)
                reg_name = REG_NAMES.get(addr, f"REG_0x{addr:02x}")
                registers[reg_name] = f"0x{value:02x}"
                
            # Print registers
            self.log("Register dump:")
            for name, value in registers.items():
                self.log(f"  {name.ljust(20)}: {value}")
                
            # Special register analysis
            op_mode = self.lora.read_register(REG_OP_MODE)
            self.log("\nDetailed register analysis:")
            
            # OP_MODE analysis
            mode_names = {
                0x00: "SLEEP",
                0x01: "STDBY",
                0x02: "FSTX",
                0x03: "TX",
                0x04: "FSRX",
                0x05: "RXCONTINUOUS",
                0x06: "RXSINGLE",
                0x07: "CAD"
            }
            
            mode = op_mode & 0x07
            lora_mode = (op_mode & 0x80) >> 7
            self.log(f"  OP_MODE: {'LoRa' if lora_mode else 'FSK/OOK'} mode, {mode_names.get(mode, 'UNKNOWN')} state")
            
            # Frequency analysis
            msb = self.lora.read_register(0x06)
            mid = self.lora.read_register(0x07)
            lsb = self.lora.read_register(0x08)
            freq = ((msb << 16) | (mid << 8) | lsb) * (32000000.0 / 2**19)
            self.log(f"  Frequency: {freq/1000000:.3f} MHz")
            
            # Power analysis
            pa_config = self.lora.read_register(0x09)
            pa_dac = self.lora.read_register(0x4d)
            
            output_power = pa_config & 0x0F
            pa_select = (pa_config & 0x80) >> 7
            
            if pa_select:
                if pa_dac == 0x87:
                    power = output_power + 5 + 15
                    self.log(f"  Output power: {power} dBm (PA_BOOST with high power)")
                else:
                    power = output_power + 2 + 15
                    self.log(f"  Output power: {power} dBm (PA_BOOST)")
            else:
                power = output_power - 1
                self.log(f"  Output power: {power} dBm (RFO)")
                
            # Bandwidth, CR, SF analysis
            modem_config1 = self.lora.read_register(0x1d)
            modem_config2 = self.lora.read_register(0x1e)
            
            bw = (modem_config1 & 0xF0) >> 4
            cr = (modem_config1 & 0x0E) >> 1
            implicit_header = modem_config1 & 0x01
            
            sf = (modem_config2 & 0xF0) >> 4
            
            bw_values = {
                0: 7.8, 1: 10.4, 2: 15.6, 3: 20.8, 4: 31.25, 5: 41.7, 6: 62.5,
                7: 125, 8: 250, 9: 500
            }
            
            self.log(f"  Bandwidth: {bw_values.get(bw, 'UNKNOWN')} kHz")
            self.log(f"  Coding rate: 4/{cr+4}")
            self.log(f"  Spreading factor: {sf}")
            self.log(f"  Header mode: {'Implicit' if implicit_header else 'Explicit'}")
            
            # IRQ flags
            irq_flags = self.lora.read_register(REG_IRQ_FLAGS)
            self.log(f"  IRQ flags: 0x{irq_flags:02x}")
            if irq_flags & 0x80:
                self.log("    ✓ RX timeout")
            if irq_flags & 0x40:
                self.log("    ✓ RX done")
            if irq_flags & 0x20:
                self.log("    ✓ Payload CRC error")
            if irq_flags & 0x10:
                self.log("    ✓ Valid header")
            if irq_flags & 0x08:
                self.log("    ✓ TX done")
            if irq_flags & 0x04:
                self.log("    ✓ CAD done")
            if irq_flags & 0x02:
                self.log("    ✓ FHSS change channel")
            if irq_flags & 0x01:
                self.log("    ✓ CAD detected")
                
            # FIFO and packet info
            payload_length = self.lora.read_register(0x22)
            self.log(f"  Payload length: {payload_length} bytes")
            
            rx_bytes = self.lora.read_register(0x13)
            self.log(f"  Last RX bytes: {rx_bytes}")
            
            # Sync word
            sync_word = self.lora.read_register(0x39)
            self.log(f"  Sync word: 0x{sync_word:02x}")
            
            # Return to STDBY mode
            self.lora.idle()
            
            # Close LoRa
            self.lora.close()
            self.lora = None
            return True
            
        except Exception as e:
            self.log(f"Register dump failed: {e}", "ERROR")
            if self.lora:
                self.lora.close()
                self.lora = None
            return False
            
    def ping_test(self):
        """Send a ping and wait for response"""
        self.log("Starting ping test...", "TEST")
        
        try:
            self.lora = LoRa(
                frequency=self.args.freq,
                bandwidth=self.args.bw,
                spreading_factor=self.args.sf,
                coding_rate=self.args.cr,
                tx_power=self.args.power,
                verbose=False
            )
            
            # Prepare ping data
            ping_data = f"PING_{datetime.now().strftime('%H%M%S')}"
            self.log(f"Sending ping: {ping_data}")
            
            # Send ping
            self.lora.send(ping_data.encode())
            self.log("Ping sent, waiting for response...")
            
            # Wait for response
            start_time = time.time()
            timeout = 30  # 30 seconds timeout
            
            while time.time() - start_time < timeout:
                sys.stdout.write(".")
                sys.stdout.flush()
                
                payload, rssi = self.lora.receive(timeout=1000)
                
                if payload:
                    try:
                        message = payload.decode('utf-8')
                        self.log(f"\nReceived response: {message}")
                        self.log(f"RSSI: {rssi} dBm")
                        self.log(f"Response time: {(time.time() - start_time):.2f} seconds")
                        break
                    except UnicodeDecodeError:
                        self.log(f"\nReceived binary data: {payload.hex()}")
                        self.log(f"RSSI: {rssi} dBm")
                        self.log(f"Response time: {(time.time() - start_time):.2f} seconds")
                        break
            else:
                self.log("\nNo response received within timeout period", "WARNING")
                
            # Close LoRa
            self.lora.close()
            self.lora = None
            return True
            
        except Exception as e:
            self.log(f"Ping test failed: {e}", "ERROR")
            if self.lora:
                self.lora.close()
                self.lora = None
            return False
            
    def freq_sweep(self):
        """Sweep through frequencies looking for activity"""
        self.log("Starting frequency sweep...", "TEST")
        
        try:
            # Define frequency range to sweep
            start_freq = self.args.freq - 5
            end_freq = self.args.freq + 5
            step = 0.1
            
            current_freq = start_freq
            results = {}
            
            self.lora = LoRa(
                frequency=start_freq,
                bandwidth=self.args.bw,
                spreading_factor=self.args.sf,
                coding_rate=self.args.cr,
                tx_power=self.args.power,
                verbose=False
            )
            
            self.log(f"Sweeping from {start_freq} MHz to {end_freq} MHz in {step} MHz steps")
            
            while current_freq <= end_freq:
                self.log(f"Testing frequency: {current_freq} MHz")
                
                # Set new frequency
                self.lora.set_frequency(current_freq)
                
                # Enter RXCONTINUOUS mode
                self.lora.write_register(REG_OP_MODE, 0x85)  # LoRa mode + RX continuous
                
                # Clear IRQ flags
                self.lora.write_register(REG_IRQ_FLAGS, 0xFF)
                
                # Wait for a moment
                time.sleep(1)
                
                # Read RSSI
                rssi_reg = self.lora.read_register(0x1B)  # RSSI value (LoRa)
                rssi = -157 + rssi_reg
                
                self.log(f"  RSSI: {rssi} dBm")
                results[current_freq] = rssi
                
                # Move to next frequency
                current_freq += step
                
            # Return to standby mode
            self.lora.idle()
            
            # Find best frequency (highest RSSI)
            best_freq = max(results, key=results.get)
            best_rssi = results[best_freq]
            
            self.log("\nFrequency sweep results:")
            self.log(f"Best frequency: {best_freq} MHz (RSSI: {best_rssi} dBm)")
            
            # Plot a simple ASCII graph
            self.log("\nRSSI Graph:")
            min_rssi = min(results.values())
            max_rssi = max(results.values())
            range_rssi = max_rssi - min_rssi
            
            for freq in sorted(results.keys()):
                rssi = results[freq]
                bars = int(((rssi - min_rssi) / max(1, range_rssi)) * 40)
                self.log(f"{freq:6.1f} MHz | {rssi:4d} dBm | {'#' * bars}")
                
            # Save results to file
            with open('freq_sweep.json', 'w') as f:
                json.dump({str(k): v for k, v in results.items()}, f)
                
            self.log("Frequency sweep data saved to freq_sweep.json")
            
            # Close LoRa
            self.lora.close()
            self.lora = None
            return True
            
        except Exception as e:
            self.log(f"Frequency sweep failed: {e}", "ERROR")
            if self.lora:
                self.lora.close()
                self.lora = None
            return False
            
    def scan_mode(self):
        """Scan for LoRa activity"""
        self.log("Starting LoRa activity scan...", "TEST")
        
        try:
            self.lora = LoRa(
                frequency=self.args.freq,
                bandwidth=self.args.bw,
                spreading_factor=self.args.sf,
                coding_rate=self.args.cr,
                tx_power=self.args.power,
                verbose=False
            )
            
            # Setup for scanning
            scan_duration = 120  # 2 minutes scan
            self.log(f"Scanning for LoRa activity on {self.args.freq} MHz for {scan_duration} seconds...")
            
            # Enter RX mode
            self.lora.write_register(REG_OP_MODE, 0x85)  # LoRa mode + RX continuous
            
            # Clear IRQ flags
            self.lora.write_register(REG_IRQ_FLAGS, 0xFF)
            
            start_time = time.time()
            packet_count = 0
            
            # Statistics
            rssi_values = []
            snr_values = []
            packet_lengths = []
            
            while time.time() - start_time < scan_duration:
                # Show time remaining
                elapsed = time.time() - start_time
                remaining = scan_duration - elapsed
                if int(remaining) % 10 == 0:
                    sys.stdout.write(f"\rScanning... {int(remaining)} seconds remaining")
                    sys.stdout.flush()
                
                # Check if we received a packet
                irq_flags = self.lora.read_register(REG_IRQ_FLAGS)
                
                if irq_flags & 0x40:  # RX_DONE flag
                    packet_count += 1
                    
                    # Get packet info
                    rx_bytes = self.lora.read_register(0x13)
                    rssi_reg = self.lora.read_register(0x1A)
                    snr_reg = self.lora.read_register(0x19)
                    
                    # Calculate RSSI and SNR
                    rssi = -157 + rssi_reg
                    snr = snr_reg / 4
                    
                    rssi_values.append(rssi)
                    snr_values.append(snr)
                    packet_lengths.append(rx_bytes)
                    
                    self.log(f"\nPacket detected! RSSI: {rssi} dBm, SNR: {snr:.1f} dB, Length: {rx_bytes} bytes")
                    
                    # Clear IRQ flags
                    self.lora.write_register(REG_IRQ_FLAGS, 0xFF)
                
                time.sleep(0.1)
                
            # Print summary
            self.log(f"\nScan complete. Detected {packet_count} packets.")
            
            if packet_count > 0:
                avg_rssi = sum(rssi_values) / len(rssi_values)
                avg_snr = sum(snr_values) / len(snr_values)
                avg_length = sum(packet_lengths) / len(packet_lengths)
                
                self.log(f"Average RSSI: {avg_rssi:.1f} dBm")
                self.log(f"Average SNR: {avg_snr:.1f} dB")
                self.log(f"Average packet length: {avg_length:.1f} bytes")
            
            # Close LoRa
            self.lora.close()
            self.lora = None
            return True
            
        except Exception as e:
            self.log(f"Activity scan failed: {e}", "ERROR")
            if self.lora:
                self.lora.close()
                self.lora = None
            return False
    
    def run_all_tests(self):
        """Run all tests sequentially"""
        self.log("Running all diagnostic tests...", "TEST")
        
        results = []
        
        self.log("\n===== HARDWARE TEST =====")
        results.append(("Hardware Test", self.test_hardware()))
        
        self.log("\n===== REGISTER DUMP =====")
        results.append(("Register Dump", self.dump_registers()))
        
        self.log("\n===== FREQUENCY SWEEP =====")
        results.append(("Frequency Sweep", self.freq_sweep()))
        
        self.log("\n===== ACTIVITY SCAN =====")
        results.append(("Activity Scan", self.scan_mode()))
        
        self.log("\n===== PING TEST =====")
        results.append(("Ping Test", self.ping_test()))
        
        # Print summary
        self.log("\n===== TEST SUMMARY =====")
        for name, result in results:
            status = "PASSED" if result else "FAILED"
            self.log(f"{name}: {status}")
            
        return all(result for _, result in results)
        
    def cleanup(self):
        """Clean up resources"""
        if self.lora:
            self.lora.close()
            self.lora = None
        self.save_log()
        
def main():
    parser = argparse.ArgumentParser(description='LoRa Debug and Test Tool')
    parser.add_argument('--mode', choices=['all', 'hardware', 'registers', 'ping', 'sweep', 'scan'], 
                        default='all', help='Test mode')
    parser.add_argument('--freq', type=float, default=433.0,
                        help='Frequency in MHz (default: 433.0)')
    parser.add_argument('--bw', type=int, default=125000,
                        help='Bandwidth in Hz (default: 125000)')
    parser.add_argument('--sf', type=int, default=12,
                        help='Spreading Factor (default: 12)')
    parser.add_argument('--cr', type=int, default=5,
                        help='Coding Rate denominator (default: 5, which is 4/5)')
    parser.add_argument('--power', type=int, default=17,
                        help='TX Power in dBm (default: 17)')
    parser.add_argument('--logfile', type=str, default='lora_debug.log',
                        help='Log file name (default: lora_debug.log)')
    
    args = parser.parse_args()
    
    print("LoRa Debug and Test Tool")
    print("=======================")
    print(f"Mode: {args.mode}")
    print(f"Frequency: {args.freq} MHz")
    print(f"Bandwidth: {args.bw} Hz")
    print(f"Spreading Factor: {args.sf}")
    print(f"Coding Rate: 4/{args.cr}")
    print(f"TX Power: {args.power} dBm")
    print("=======================")
    
    debug = LoRaDebug(args)
    
    try:
        if args.mode == 'all':
            debug.run_all_tests()
        elif args.mode == 'hardware':
            debug.test_hardware()
        elif args.mode == 'registers':
            debug.dump_registers()
        elif args.mode == 'ping':
            debug.ping_test()
        elif args.mode == 'sweep':
            debug.freq_sweep()
        elif args.mode == 'scan':
            debug.scan_mode()
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        debug.cleanup()
        
if __name__ == "__main__":
    main()
