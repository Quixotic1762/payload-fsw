#!/usr/bin/env python3

import subprocess
import csv
import os
import sys
import signal

# === CONFIGURATION ===
# Frequency range: lower:upper in MHz (as floats)
FREQ_LO = 700.0     # e.g., 433 MHz
FREQ_HI = 705.0 # e.g., 439 MHz
BIN_WIDTH_HZ = 25000  # 25 kHz bins, for example
GAIN_DB = 40        # fixed gain (helps consistency)
INTEGRATION_S = 0.5 # seconds per sweep
CSV_FILE = "test_logs/rtl_rssi_log.csv"

# Ensure directory exists
os.makedirs(os.path.dirname(CSV_FILE), exist_ok=True)

def termination_handler(signum, frame):
    sys.exit()
    f.close()

def run_sweep():
    global FREQ_LO
    global FREQ_HI
    # Write header if new file
    with open(CSV_FILE, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Timestamp","Freq_MHz","RSSI_dBm"])
    # Build rtl_power command
    
    
    # Run the process
    while True:
        FREQ_LO += 5
        FREQ_HI += 5
        if (FREQ_LO >= 2075) and (FREQ_HI >= 2080):
            FREQ_LO = 700
            FREQ_HI = 705
        freq_range_arg = "{:.6f}M:{:.6f}M:{:.0f}".format(FREQ_LO, FREQ_HI, BIN_WIDTH_HZ)
        cmd = [
        "rtl_power",
        "-f", freq_range_arg,
        "-g", str(GAIN_DB),
        "-i", str(INTEGRATION_S),
        "-1",   # single-shot mode
        "-"     # output to stdout
        ]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        except Exception as e:
            print("Error starting rtl_power:", e, file=sys.stderr)
            return

        f = open(CSV_FILE, "a", newline="")
        w = csv.writer(f)
        for line in proc.stdout:
            # Example line: 2025-10-18,17:44:33,433000000,439000000,25000,40,-60.24,-58.70,...
            parts = line.strip().split(",")
            if len(parts) < 7:
                continue
            timestamp = "{} {}".format(parts[0], parts[1])
            try:
                hz_low = float(parts[2])
                bin_w = float(parts[4])
                rssi_vals = [ float(r) for r in parts[6:]]
            except ValueError:
                continue

            for i, r in enumerate(rssi_vals):
                freq_mhz = (hz_low + i*bin_w) / 1e6
                # Log to CSV and print
                r += 5
                w.writerow([timestamp, "{:.3f}".format(freq_mhz), r])
                f.flush()
                print(f"{timestamp} | {freq_mhz:.3f} MHz | {r:.2f} dBm")
        
        # Wait a bit before next sweep if you loop later
        #time.sleep(0.1)

if __name__ == "__main__":
    run_sweep()
