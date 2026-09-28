"""
Minimal diagnostic script for LiteWing drone.
Run this ALONE (no GUI script running) to check:
1. Can we connect to the drone?
2. What log variables does the firmware expose?
3. Can we read sensor data?

Usage:
  python diagnose_drone.py
"""
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

DRONE_URI = "udp://192.168.43.42"

def main():
    print("=" * 60)
    print("LiteWing Drone Diagnostic Tool")
    print("=" * 60)
    
    # Step 1: Init drivers
    print("\n[1] Initializing cflib CRTP drivers...")
    cflib.crtp.init_drivers()
    print("    OK: Drivers initialized")
    
    # Step 2: Connect
    print(f"\n[2] Connecting to drone at {DRONE_URI}...")
    print("    (Make sure your laptop is connected to the LiteWing WiFi!)")
    
    cf = Crazyflie(rw_cache="./cache")
    
    try:
        with SyncCrazyflie(DRONE_URI, cf=cf) as scf:
            print("    OK: Connected to drone!")
            
            # Step 3: Read the TOC (Table of Contents)
            print("\n[3] Reading log TOC (available variables)...")
            toc = cf.log.toc.toc
            
            if not toc:
                print("    ERROR: TOC is empty! No log variables available.")
                print("    This means the firmware isn't exposing any sensor data.")
                return
            
            print(f"    OK: Found {len(toc)} log groups:")
            for group_name in sorted(toc.keys()):
                vars_in_group = list(toc[group_name].keys())
                print(f"      - {group_name}: {vars_in_group}")
            
            # Step 4: Check for critical variables
            print("\n[4] Checking for critical variables...")
            critical_vars = [
                ("motion", "deltaX"),
                ("motion", "deltaY"),
                ("stateEstimate", "z"),
                ("range", "zrange"),
                ("pm", "vbat"),
                ("sensor", "pmw3901"),
            ]
            
            for group, name in critical_vars:
                if group in toc and name in toc[group]:
                    print(f"    OK: {group}.{name} FOUND")
                else:
                    if group not in toc:
                        print(f"    MISSING: {group}.{name} (group '{group}' not in TOC!)")
                    else:
                        print(f"    MISSING: {group}.{name} (name '{name}' not in group '{group}')")
            
            # Step 5: Try to read sensor data
            print("\n[5] Trying to read motion data for 5 seconds...")
            
            # Find available motion variables
            available_vars = []
            for var_name, var_type in [("motion.deltaX", "int16_t"), ("motion.deltaY", "int16_t"), 
                                        ("stateEstimate.z", "float"), ("range.zrange", "uint16_t")]:
                group, name = var_name.split(".")
                if group in toc and name in toc[group]:
                    available_vars.append((var_name, var_type))
            
            if not available_vars:
                print("    ERROR: No motion variables available to subscribe to!")
                print("    The PMW3901 flow sensor or VL53L1X range sensor may not be initialized.")
                
                # Check param for pmw3901 status
                print("\n[5b] Checking sensor parameters...")
                try:
                    toc_params = cf.param.toc.toc
                    if "sensor" in toc_params:
                        print(f"    sensor params: {list(toc_params['sensor'].keys())}")
                        for pname in toc_params['sensor']:
                            try:
                                val = cf.param.get_value(f"sensor.{pname}")
                                print(f"      sensor.{pname} = {val}")
                            except Exception as e:
                                print(f"      sensor.{pname} = ERROR: {e}")
                    else:
                        print("    No 'sensor' param group found")
                    
                    # Print all param groups for info
                    print(f"\n    All param groups: {sorted(toc_params.keys())}")
                except Exception as e:
                    print(f"    Error reading params: {e}")
                return
            
            print(f"    Subscribing to: {[v[0] for v in available_vars]}")
            
            received_data = []
            
            def data_callback(timestamp, data, logconf):
                received_data.append(data)
                print(f"    DATA @ {timestamp}: {data}")
            
            log_conf = LogConfig(name="Diag", period_in_ms=100)
            for var_name, var_type in available_vars:
                log_conf.add_variable(var_name, var_type)
            
            log_conf.data_received_cb.add_callback(data_callback)
            cf.log.add_config(log_conf)
            
            if not log_conf.valid:
                print("    ERROR: Log configuration is INVALID!")
                return
            
            log_conf.start()
            print("    Logging started, waiting for data...")
            
            time.sleep(5.0)
            
            log_conf.stop()
            
            if received_data:
                print(f"\n    SUCCESS: Received {len(received_data)} data packets!")
            else:
                print(f"\n    ERROR: No data received in 5 seconds!")
                print("    This usually means the sensor hardware is not responding.")
            
            # Step 6: Check battery
            print("\n[6] Checking battery...")
            battery_vars = []
            if "pm" in toc and "vbat" in toc["pm"]:
                log_bat = LogConfig(name="Bat", period_in_ms=500)
                log_bat.add_variable("pm.vbat", "float")
                
                bat_data = []
                def bat_callback(timestamp, data, logconf):
                    bat_data.append(data)
                
                log_bat.data_received_cb.add_callback(bat_callback)
                cf.log.add_config(log_bat)
                if log_bat.valid:
                    log_bat.start()
                    time.sleep(2.0)
                    log_bat.stop()
                    if bat_data:
                        print(f"    Battery: {bat_data[-1].get('pm.vbat', 'N/A')}V")
                    else:
                        print("    No battery data received")
                else:
                    print("    Battery log config invalid")
            else:
                print("    pm.vbat not in TOC")
            
            print("\n" + "=" * 60)
            print("DIAGNOSTIC COMPLETE")
            print("=" * 60)
            
    except Exception as e:
        print(f"\n    FAILED TO CONNECT: {e}")
        print("\n    Troubleshooting:")
        print("    1. Is your laptop connected to the LiteWing WiFi network?")
        print("    2. Is the drone powered on?")
        print("    3. Can you ping 192.168.43.42?")
        print("    4. Is another Python script already running?")

if __name__ == "__main__":
    main()
