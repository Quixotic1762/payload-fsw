import subprocess
import serial
import time
import ina219_lib

blenano_proc_log = 'proc_files/blenano_proc_log'
blevsas_log = 'proc_files/blevsas_log'
gnss_proc_log = 'proc_files/gnss_proc_log'
hackrf_log = 'proc_files/hackrf_log'
telemetry_log = 'telemetry_logs/telemetry_log'

gsm_serial = serial.Serial("/dev/ttyAMA2", 9600, timeout=1)
gnss_serial = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)
nano_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
try: 
    nano_stdout = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
except:
    pass

def lora(telm_q, tx_enable,global_packet_count):
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
                spreading_factor=11,
                coding_rate=5,
                tx_power=17,
                verbose=False
            )
            #lora = LoRa()
            counter = 0
            while True:
                if (telm_q.qsize() > 0):
                    message = telm_q.get()
                    message = f"{message}"
                    lora.send(message.encode())
                    global_packet_count.value += 1
                    #print(message)
                #payload, rssi = lora.receive(timeout=50)
                '''
                payload = 0
                if payload:
                    print(payload)
                    print(payload.decode('utf-8'))
                    if payload.decode('utf-8') == '1':
                        tx_enable.set()
                '''
    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if 'lora' in locals():
                lora.close()
                print("LoRa resources released.")
    main()

def hackrf_proc():
    import subprocess
    import csv
    import datetime
    import time
    
    def run_hackrf_sweep():
        with open(hackrf_log, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Frequency_MHz", "RSSI_dBm"])

            process = subprocess.Popen(["hackrf_sweep", "-f", "700:2700"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            for line in process.stdout: 
                parts = line.strip().split(",")
                if len(parts) >= 7: 
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    try:
                        hz_low = int(parts[2])
                        hz_high = int(parts[3])
                        bin_width = float(parts[4])
                        rssi_values = parts[6:]

                        for i, rssi in enumerate(rssi_values):
                            freq_mhz = (hz_low + i * bin_width) / 1e6
                            writer.writerow([timestamp, freq_mhz, float(rssi)])
                            file.flush()
                            #print(f"{timestamp} | {freq_mhz:.2f} MHz | {rssi} dBm")0000000000000000000
                        time.sleep(0.1)

                    except ValueError:
                        continue

    run_hackrf_sweep()

'''
Author: Ashwin Kumar, Ghanit Taunk
'''
def gnss_proc():
    import serial
    import csv

    ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)
    gnss_proc_log = 'proc_files/gnss_proc_log'

    def decimal(coord, direction):
        if not coord:
            return None
        try:
            degrees = int(coord[:2])
            minutes = float(coord[2:])
            decimal = degrees + (minutes / 60)
            if direction in ["S", "W"]:
                decimal *= -1
            return decimal
        except ValueError:
            return None
        
    def decimal_long(coord, direction):
        if not coord:
            return None
        try:
            degrees = int(coord[:3])
            minutes = float(coord[3:])
    
            decimal = degrees + (minutes / 60)
            if direction in ["S", "W"]:
                decimal *= -1
            return decimal
        except ValueError:
            return None 
    #print(decimal)

    altitude = None
    num_sats = None
    
    with open(gnss_proc_log, "w", newline="") as gnss_fd:
        writer = csv.writer(gnss_fd)
        writer.writerow(["UTC","Latitude","Longitude","Sats","Altitude"])
        
        while True:
            raw = ser.readline().decode("utf-8", errors='ignore').strip()
            #print(raw)
            
            if raw.startswith("$GNGGA"):
                f_raw = raw.split(",")
                if len(f_raw) > 9:
                    if f_raw[9]:
                        try:
                            altitude = float(f_raw[9])
                        except ValueError:
                            altitude = None
                    if f_raw[7]:
                        try:
                            num_sats = int(f_raw[7])
                        except ValueError:
                            num_sats = None

            if raw.startswith("$GNRMC"):
                f_raw = raw.split(",")
                if len(f_raw) > 6:
                    lat = f_raw[3]
                    lat_dir = f_raw[4]
                    dec_lat = decimal(lat, lat_dir)
                    long = f_raw[5]
                    long_dir = f_raw[6]
                    dec_long = decimal_long(long, long_dir)
                writer.writerow([f_raw[1],dec_lat, dec_long, num_sats, altitude])


def blenano_proc(arduino_flag):
    import serial
    import csv
    import os

    nano_serial.reset_input_buffer()

    ble_file = "proc_files/blenano_proc_log"
    volt_file = "proc_files/blevsas_log"
    
    with open(ble_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp", "Temperature", "Roll", "Pitch", "Yaw",
            "Pressure", "Altitude", "ax", "ay", "az",
            "gx", "gy", "gz", "mx", "my", "mz"
        ])

    with open(volt_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Voltage"])

    buffer = ""
    while True:
        try:
            data = nano_serial.read(nano_serial.in_waiting or 1).decode("utf-8", errors="ignore").strip()
            buffer += data

            while "<" in buffer and ">" in buffer:
                start = buffer.find("<")
                end = buffer.find(">", start)
                if end == -1:
                    break  

                packet = buffer[start+1:end]  
                buffer = buffer[end+1:]

                parts = packet.split(",")
                if not parts:
                    continue

                if parts[0] == "V" and len(parts) == 3:
                    _, ts, volt = parts
                    with open(volt_file, "a", newline="") as vf:
                        writer = csv.writer(vf)
                        writer.writerow([ts, volt])

                elif parts[0] == "S" and len(parts) == 17:
                    _, *sensor_data = parts
                    with open(ble_file, "a", newline="") as sf:
                        writer = csv.writer(sf)
                        writer.writerow(sensor_data)

        except Exception as e:
            arduino_flag.clear()
            continue



def gnss_proc():
    import serial
    import csv

    ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)
    gnss_proc_log = 'proc_files/gnss_proc_log'

    def decimal(coord, direction):
        if not coord:
            return None
        try:
            degrees = int(coord[:2])
            minutes = float(coord[2:])
            decimal = degrees + (minutes / 60)
            if direction in ["S", "W"]:
                decimal *= -1
            return decimal
        except ValueError:
            return None
        
    def decimal_long(coord, direction):
        if not coord:
            return None
        try:
            degrees = int(coord[:3])
            minutes = float(coord[3:])
    
            decimal = degrees + (minutes / 60)
            if direction in ["S", "W"]:
                decimal *= -1
            return decimal
        except ValueError:
            return None 
    #print(decimal)

    altitude = None
    num_sats = None
    
    with open(gnss_proc_log, "w", newline="") as gnss_fd:
        writer = csv.writer(gnss_fd)
        writer.writerow(["UTC","Latitude","Longitude","Sats","Altitude"])
        
        while True:
            raw = ser.readline().decode("utf-8", errors='ignore').strip()
            #print(raw)
            
            if raw.startswith("$GNGGA"):
                f_raw = raw.split(",")
                if len(f_raw) > 9:
                    if f_raw[9]:
                        try:
                            altitude = float(f_raw[9])
                        except ValueError:
                            altitude = None
                    if f_raw[7]:
                        try:
                            num_sats = int(f_raw[7])
                        except ValueError:
                            num_sats = None


            if raw.startswith("$GNRMC"): 
                f_raw = raw.split(",")
                if len(f_raw) > 6:
                    lat = f_raw[3]
                    lat_dir = f_raw[4]
                    dec_lat = decimal(lat, lat_dir)
                    long = f_raw[5]
                    long_dir = f_raw[6]
                    dec_long = decimal_long(long, long_dir)
                writer.writerow([f_raw[1],dec_lat, dec_long, num_sats, altitude])
                gnss_fd.flush()


def gsm_proc(sms_q):
    import serial 
    import time

    last_transmit = time.time()

    def send_at(command):
        #tr_buffer = (command+'\r\n').encode(encoding="utf-8")
        gsm_serial.write((command+'\r\n').encode())
        while gsm_serial.readline():
            print(gsm_serial.readline().decode('utf-8'))
    
    def send_sms(number, message):
        send_at("AT+CMGF=1")
        set_nu = 'AT+CMGS="'+number+'"'
        send_at(set_nu)
        msg = message + chr(26)
        msg_bytes = msg.encode()
        gsm_serial.write(msg_bytes)
    
    while True:
        curr_time = time.time()
        if (curr_time - last_transmit) and (sms_q.qsize > 0):
            sms_payload = sms_q.get()
            send_sms(number, sms_payload)
        time.sleep(0.1)

def check_arduino_health(arduino_flag):
    nano_alive = '3'
    global nano_stdout
    global nano_serial
    dev = 0
    reinit_flag = 0
    while True:
        if reinit_flag:
            try:
                dev = 1 - dev
                dev_path = f"/dev/ttyACM{dev}"
                nano_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
                nano_stdout = serial.Serial(dev_path, 115200, timeout=1)
                reinit_flag = 0
            except:
                reinit_flag = 1 
                arduino_flag.clear()
                pass
             
        try:
            nano_serial.write(nano_alive.encode())
            buffer = nano_stdout.read(nano_stdout.in_waiting).decode('utf-8')
        except OSError:
            buffer = '0'
            arduino_flag.clear()
            reinit_flag = 1

        if "128" in buffer:
            reinit_flag = 0
            arduino_flag.set()
        time.sleep(2)

def check_rpi_temp():                                                            
    rpi_temp = subprocess.check_output(['vcgencmd', 'measure_temp'])
    rpi_temp = rpi_temp.decode()
    rpi_temp = rpi_temp[:-1]
    rpi_temp = rpi_temp.split('=')[1][:-2]
    return (rpi_temp)


def measure_voltage(current_shared, voltage_shared, electrical_health_lock):
    ina219 = ina219_lib.INA219(i2c_bus=1,addr=0x43)
    while True:
        with electrical_health_lock:
            voltage = float(ina219.getBusVoltage_V())
            voltage = float(f"{voltage:.3f}")
            voltage_shared.value = voltage
            current = float(ina219.getCurrent_mA())
            current = current/1000
            current = float(f"{current:.3f}")
            current_shared.value = current
            #print(f"Voltage: {voltage_shared.value:.3f}, current: {current_shared.value:.3f}")
        time.sleep(0.2)

def health_check(temp_event, voltage_event, ocp_event, current_shared, voltage_shared, electrical_health_lock):
    vol_curr = 0
    while True:
        rpi_temp = float(check_rpi_temp())
        with electrical_health_lock:
            vol_curr = [voltage_shared.value, current_shared.value]
        if (rpi_temp > 90):
            # set event
            temp_event.set()
        if (vol_curr[0] < 3.15):
            #set event
            voltage_event.set()
        if (vol_curr[1] > 3):
            ocp_event.set()
        else:
            ocp_event.clear()
        time.sleep(0.5)

def rpicam_proc(pid_shared):
    import os
    import datetime
    import subprocess
    import threading
    import signal
    import time
    import cv2
    from PIL import Image
    import piexif
    import csv

    def analyze_frame(frame, prev_frame):
        results = {}
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results["mean_brightness"] = round(float(gray.mean()), 2)
    
        # Ground vs Sky ratio
        h, w = gray.shape
        results["sky_ratio"] = "More sky" if gray[:h//2, :].mean() > gray[h//2:, :].mean() else "More ground"
    
        # Cloud cover %
        results["cloud_cover_percent"] = round(100 * (gray > 180).sum() / gray.size, 2)
    
        # Color histogram sum
        hist = cv2.calcHist([frame], [0], None, [32], [0, 256])
        results["histogram_sum"] = int(hist.sum())
    
        # Anomaly detection
        if prev_frame is not None:
            diff = cv2.absdiff(cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY), gray)
            results["anomaly"] = f"?? Sudden change (score={diff.mean():.2f})" if diff.mean() > 25 else "Normal"
        else:
            results["anomaly"] = "First frame"
    
        return results
    
    def save_with_exif(frame_path, frame_number, folder_name, results):
        summary = "; ".join([f"{k}:{v}" for k, v in results.items()])
        timestamp = datetime.datetime.now().strftime("%Y:%m:%d %H:%M:%S")
    
        exif_dict = {
            "0th": {
                piexif.ImageIFD.Make: u"Raspberry Pi",
                piexif.ImageIFD.Model: u"PiCam v3",
                piexif.ImageIFD.Software: u"Mission Logger",
                piexif.ImageIFD.ImageDescription: f"Frame {frame_number}, {folder_name}, {summary}",
            },
            "Exif": {
                piexif.ExifIFD.DateTimeOriginal: timestamp,
                piexif.ExifIFD.UserComment: summary.encode("utf-8"),
            },
        }
    
        exif_bytes = piexif.dump(exif_dict)
        img = Image.open(frame_path)
        img.save(frame_path, exif=exif_bytes)

    class VideoRecorder(threading.Thread):
        def __init__(self, video_path):
            threading.Thread.__init__(self, group=None)
            self.video_path = video_path
            self.process = None

        def run(self):
            self.process = subprocess.Popen([
                "rpicam-vid",
                "-t", "0",                     # record until stopped
                "-o", self.video_path,         # output file
                "--framerate", "60",           # FPS
                "--width", "1920",             # width
                "--height", "1080",            # height
                "--nopreview"                  # headless
            ])
            self.process.wait()

        def stop(self):
            if self.process:
                # Send SIGINT instead of terminate for proper MP4 finalization
                self.process.send_signal(signal.SIGINT)
                self.process.wait()
                time.sleep(1)  # allow libcamera to finalize file


    def main():
        pid_shared.value = os.getpid()
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        vid_dir_list = sorted(os.listdir('vid_logs/'))
        try:
            last_vid = vid_dir_list[-1]
        except IndexError:
            last_vid = 'mission_0'

        last_vid_arr = last_vid.split('_')
        print(last_vid_arr)
        last_vid_arr[1] = str(int(last_vid_arr[1]) + 1)
        folder = '_'.join(last_vid_arr)
        folder = f"vid_logs/{folder}"

        os.makedirs(folder, exist_ok=True)

        video_path = os.path.join(folder, "output.mp4")
        report_path = os.path.join(folder, "image_analysis_report.txt")
        frames_dir = os.path.join(folder, "frames")
        os.makedirs(frames_dir, exist_ok=True)

        recorder = VideoRecorder(video_path)
        recorder.start()
        print("? Recording started. Press CTRL+C to stop...")

        # Wait until user stops recording
        try:
            signal.pause()
        except KeyboardInterrupt:
            print("\n? Stopping recording...")
            recorder.stop()
        
    # Extract frames at 1 FPS using ffmpeg
        print("? Extracting frames at 1 FPS...")
        subprocess.run([
            "ffmpeg", "-i", video_path, "-vf", "fps=1",
            os.path.join(frames_dir, "frame_%04d.jpg")
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
        # -------------------------------
        # Log frame timestamps
        # -------------------------------
        frame_files = sorted([f for f in os.listdir(frames_dir) if f.endswith(".jpg")])
        start_time = datetime.datetime.now()  # use extraction time as base
        with open(os.path.join(frames_dir, "frame_timestamps.csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["frame", "timestamp"])
            for i, frame in enumerate(frame_files):
                timestamp = start_time + datetime.timedelta(seconds=i)  # 1 FPS
                writer.writerow([frame, timestamp.strftime("%Y-%m-%d %H:%M:%S")])
    
        # Analyze frames
        prev_frame = None
        with open(report_path, "w") as report:
            for i, f_name in enumerate(frame_files):
                frame_path = os.path.join(frames_dir, f_name)
                frame = cv2.imread(frame_path)
                results = analyze_frame(frame, prev_frame)
                report.write(f"{f_name} ? {results}\n")
                report.flush()
                save_with_exif(frame_path, i, folder, results)
                prev_frame = frame
    
        print(f"? Mission data saved in {folder}")


    main()

def ocp_shutdown(ocp_event):
    ocp_event.wait()
    print("Entered OCP")
    while True:
        print(ocp_event.is_set())
        time.sleep(2)

    # turn off hackrf
    # turn off camera (if running) 
    # reset arduino 
    # gnss?
    # gsm ? 
    
def getline(proc_fp):
    line = subprocess.check_output(['tail','-n','1',proc_fp])
    return line.decode().split('\n')[0]
