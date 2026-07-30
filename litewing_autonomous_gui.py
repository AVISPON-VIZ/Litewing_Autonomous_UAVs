"""
litewing_autonomous_gui.py
--------------------------
A simple GUI for controlling the LiteWing drone with the ESP32-CAM vision module.

The drone navigates FULLY AUTONOMOUSLY using dynamic continuous control.
It explores any indoor environment by:
  1. Flying forward with continuous speed scaling
  2. Steering toward open space with dynamic yaw rate modulation
  3. Continuous smooth deceleration as obstacles approach
  4. Rotating in place when completely blocked (to find open direction)

GUI buttons:
  - Connect / Disconnect
  - Arm (start motors)
  - Disarm (stop motors)
  - Start Autonomous (enable vision-based navigation)
  - Stop Autonomous (return to manual control)
  - Emergency Stop (cut motors immediately with 0ms delay)

Requirements:
  pip install cflib
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import sys
from cflib.crtp.crtpstack import CRTPPacket

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
except ImportError:
    print("ERROR: cflib not installed. Run: pip install cflib")
    sys.exit(1)


DRONE_URI = "udp://192.168.43.42"  # LiteWing default IP


class LiteWingGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LiteWing Dynamic Autonomous Navigation")
        self.root.geometry("580x620")

        self.cf = None
        self.log_configs = []
        self.connected = False
        self.autonomous_active = False
        self.autonomous_param_supported = False
        self.last_telemetry_seen = None
        self.last_packet_count = 0
        self.last_yaw = None
        self.last_collision = None

        self._build_ui()

    def _build_ui(self):
        # ----- Connection Frame -----
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        self.uri_var = tk.StringVar(value=DRONE_URI)
        ttk.Label(conn_frame, text="Drone URI:").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(conn_frame, textvariable=self.uri_var, width=25).grid(row=0, column=1, padx=5)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))

        # ----- Live Status Frame -----
        status_frame = ttk.LabelFrame(self.root, text="Live Drone Status & Telemetry (LOG)", padding=10)
        status_frame.pack(fill=tk.X, padx=10, pady=5)

        self.battery_var = tk.StringVar(value="Battery: --")
        self.mode_var = tk.StringVar(value="Mode: MANUAL")
        self.firmware_var = tk.StringVar(value="Firmware: CHECKING")
        self.cam_status_var = tk.StringVar(value="Camera: OFF")
        self.vision_status_var = tk.StringVar(value="Vision: OFF")
        self.yaw_var = tk.StringVar(value="Vision Yaw: --")
        self.collision_var = tk.StringVar(value="Collision Prob: --")
        self.packets_var = tk.StringVar(value="Packets: 0")
        self.avoided_var = tk.StringVar(value="Obstacles Avoided: 0")
        self.litewing_steer_var = tk.StringVar(value="Target Yaw Rate: --")
        self.litewing_fwd_vel_var = tk.StringVar(value="Target Fwd Vel (vx): --")
        self.litewing_ang_vel_var = tk.StringVar(value="Target Lat Vel (vy): --")
        self.log_status_var = tk.StringVar(value="Log Status: Discovering...")

        ttk.Label(status_frame, textvariable=self.battery_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.mode_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.firmware_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.cam_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.vision_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.yaw_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.collision_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.packets_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.avoided_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.litewing_steer_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.litewing_fwd_vel_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.litewing_ang_vel_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.log_status_var).pack(anchor=tk.W)

        # ----- Manual Control Frame -----
        manual_frame = ttk.LabelFrame(self.root, text="Manual Control", padding=10)
        manual_frame.pack(fill=tk.X, padx=10, pady=5)

        self.arm_btn = ttk.Button(manual_frame, text="Arm (Start Motors)",
                                   command=self.arm_drone, state=tk.DISABLED)
        self.arm_btn.pack(side=tk.LEFT, padx=5)

        self.disarm_btn = ttk.Button(manual_frame, text="Disarm (Stop Motors)",
                                      command=self.disarm_drone, state=tk.DISABLED)
        self.disarm_btn.pack(side=tk.LEFT, padx=5)

        # ----- Autonomous Control Frame -----
        auto_frame = ttk.LabelFrame(self.root, text="Autonomous Navigation", padding=10)
        auto_frame.pack(fill=tk.X, padx=10, pady=5)

        self.start_auto_btn = ttk.Button(auto_frame, text="START AUTONOMOUS",
                                         command=self.start_autonomous, state=tk.DISABLED)
        self.start_auto_btn.pack(side=tk.LEFT, padx=5, ipadx=10, ipady=5)

        self.stop_auto_btn = ttk.Button(auto_frame, text="STOP AUTONOMOUS",
                                        command=self.stop_autonomous, state=tk.DISABLED)
        self.stop_auto_btn.pack(side=tk.LEFT, padx=5, ipadx=10, ipady=5)

        self.auto_status_label = ttk.Label(auto_frame, text="OFF",
                                            foreground="gray", font=("Arial", 12, "bold"))
        self.auto_status_label.pack(side=tk.LEFT, padx=20)

        # ----- Emergency Stop Frame -----
        emerg_frame = ttk.LabelFrame(self.root, text="EMERGENCY", padding=10)
        emerg_frame.pack(fill=tk.X, padx=10, pady=5)

        self.emerg_btn = tk.Button(emerg_frame, text="EMERGENCY STOP (INSTANT CUT)",
                                    command=self.emergency_stop, bg="red", fg="white",
                                    font=("Arial", 14, "bold"), height=2, state=tk.DISABLED)
        self.emerg_btn.pack(fill=tk.X)

        # ----- Info Frame -----
        info_frame = ttk.LabelFrame(self.root, text="System Information", padding=10)
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        info_text = (
            "Dynamic Continuous Navigation Active:\n"
            "- Continuous quadratic pitch scaling based on obstacle proximity\n"
            "- Dynamic yaw rate gain amplification when steering near walls\n"
            "- EMA Low-Pass filtering for smooth motor execution\n"
            "- Instant Priority 0 Release on Emergency Stop (No 2s delay)\n\n"
            "Safety Check:\n"
            "Test props-OFF on bench before powered flight!"
        )
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).pack(anchor=tk.W)

        # ----- Debug Output Frame -----
        output_frame = ttk.LabelFrame(self.root, text="Debug / Telemetry Output", padding=10)
        output_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        output_frame.columnconfigure(0, weight=1)
        output_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(output_frame, height=10, wrap=tk.NONE, state=tk.DISABLED)
        self.log_text.grid(row=0, column=0, sticky=tk.NSEW)
        vscrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        vscrollbar.grid(row=0, column=1, sticky=tk.NS)
        hscrollbar = ttk.Scrollbar(output_frame, orient=tk.HORIZONTAL, command=self.log_text.xview)
        hscrollbar.grid(row=1, column=0, sticky=tk.EW)
        self.log_text.config(yscrollcommand=vscrollbar.set, xscrollcommand=hscrollbar.set)

        debug_controls = ttk.Frame(output_frame)
        debug_controls.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(5,0))
        ttk.Button(debug_controls, text="Clear Debug", command=self._clear_debug).pack(side=tk.LEFT)
        self.debug_status = tk.StringVar(value="Debug ready")
        ttk.Label(debug_controls, textvariable=self.debug_status).pack(side=tk.LEFT, padx=(10,0))

    # ============================================================
    # Connection
    # ============================================================
    def toggle_connection(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        uri = self.uri_var.get()
        self.status_label.config(text=f"Status: Connecting to {uri}...", foreground="orange")
        self.root.update()

        try:
            cflib.crtp.init_drivers()
            self.cf = Crazyflie(rw_cache="./cache")
            self.cf.connected.add_callback(self._on_connected)
            self.cf.connection_failed.add_callback(self._on_connection_failed)
            self.cf.disconnected.add_callback(self._on_disconnected)
            self.cf.open_link(uri)
            self.cam_status_var.set("Camera: OFF")
            self.debug_status.set("Connecting...")
        except Exception as e:
            self.status_label.config(text=f"Status: Error - {e}", foreground="red")
            messagebox.showerror("Error", f"Connection failed:\n{e}")

    def disconnect(self):
        if self.cf:
            for config in self.log_configs:
                try:
                    config.stop()
                except Exception:
                    pass
            self.log_configs = []
            try:
                self.cf.close_link()
            except Exception:
                pass
            self.cf = None
        self.connected = False
        self.autonomous_active = False
        self.last_telemetry_seen = None
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.firmware_var.set("Firmware: CHECKING")
        self.cam_status_var.set("Camera: OFF")
        self.debug_status.set("Debug ready")
        self.connect_btn.config(text="Connect")
        self.arm_btn.config(state=tk.DISABLED)
        self.disarm_btn.config(state=tk.DISABLED)
        self.start_auto_btn.config(state=tk.DISABLED)
        self.stop_auto_btn.config(state=tk.DISABLED)
        self.emerg_btn.config(state=tk.DISABLED)

    # ============================================================
    # Log Subscription (High-Speed Telemetry Streaming)
    # ============================================================
    def _setup_log_block(self):
        try:
            # IMPORTANT: Reset log blocks on the drone to avoid hitting MAX_BLOCKS limits
            # from previous disconnected sessions.
            if hasattr(self.cf.log, 'reset'):
                self.cf.log.reset()

            for config in self.log_configs:
                try:
                    config.stop()
                except Exception:
                    pass
            self.log_configs = []

            groups = self._discover_log_groups()
            self._append_log(f"Available log groups: {', '.join(sorted(groups.keys()))}")
            
            # Print exact variables in autonomous group to debug
            if 'autonomous' in groups:
                auto_vars = [name for name, _ in groups['autonomous']]
                self._append_log(f"DEBUG: Found autonomous variables: {', '.join(auto_vars)}")
            if 'ctrltarget' in groups:
                ctrl_vars = [name for name, _ in groups['ctrltarget']]
                self._append_log(f"DEBUG: Found ctrltarget variables: {', '.join(ctrl_vars)}")

            if 'autonomous' in groups:
                self._create_log_config(
                    "AutonomousVision",
                    [
                        ("autonomous.yaw", "float"),
                        ("autonomous.collision", "float"),
                        ("autonomous.packets", "uint32_t"),
                        ("autonomous.avoided", "uint32_t"),
                    ]
                )
            else:
                self._append_log("Warning: 'autonomous' log group not available on this firmware.")

            if 'ctrltarget' in groups:
                self._create_log_config(
                    "LiteWingControlTarget",
                    [
                        ("ctrltarget.vx", "float"),
                        ("ctrltarget.vy", "float"),
                        ("ctrltarget.yaw", "float"),
                    ]
                )
            else:
                self._append_log("No 'ctrltarget' group found. (Is this Crazyflie firmware?)")

            active_logs = [config.name for config in self.log_configs]
            if active_logs:
                self.log_status_var.set(f"Log Status: {', '.join(active_logs)}")
                self._append_log(f"Started log configs: {', '.join(active_logs)}")
            else:
                self.log_status_var.set("Log Status: NONE")

            self.root.after(2500, self._check_camera_state)
        except Exception as e:
            err_msg = f"Log subscription failed: {e}\nVerify the LiteWing firmware has the expected log groups!"
            self._append_log(err_msg)
            self.status_label.config(text="Status: Log subscription failed", foreground="red")
            messagebox.showerror("Firmware Mismatch", err_msg)
            if hasattr(self, 'cf') and self.cf and self.cf.log and self.cf.log.toc:
                groups = set([var.group for var in self.cf.log.toc.toc.values()])
                self._append_log(f"Available log groups on drone: {', '.join(sorted(groups))}")

    def _discover_log_groups(self):
        groups = {}
        try:
            if self.cf and self.cf.log and self.cf.log.toc:
                for var in self.cf.log.toc.toc.values():
                    groups.setdefault(var.group, []).append((var.name, var.ctype))
        except Exception as e:
            print(f"Exception in _discover_log_groups: {e}")
        return groups

    def _create_log_config(self, name, variables):
        log_config = LogConfig(name=name, period_in_ms=100)
        for var_name, var_type in variables:
            log_config.add_variable(var_name, var_type)
        self.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self._log_data_received)
        log_config.error_cb.add_callback(self._log_error)
        log_config.start()
        self.log_configs.append(log_config)
        return log_config

    def _log_error(self, logconf, msg):
        self._append_log(f"LOG ERROR for {logconf.name}: {msg}")
        print(f"LOG ERROR for {logconf.name}: {msg}")

    def _log_data_received(self, timestamp, data, logconf):
        print(f"[DEBUG] _log_data_received CALLED! logconf={logconf.name} data: {data}")
        if logconf.name == "AutonomousVision":
            self._handle_xiao_telemetry(data)
        elif logconf.name == "LiteWingControlTarget":
            self._handle_litewing_control(data)
        else:
            self._append_log(f"Unknown log source '{logconf.name}' received: {data}")

    def _handle_xiao_telemetry(self, data):
        yaw = data.get("autonomous.yaw", 0.0)
        coll = data.get("autonomous.collision", 0.0)
        packets = data.get("autonomous.packets", 0)
        avoided = data.get("autonomous.avoided", 0)
        has_payload = any(key in data for key in ("autonomous.yaw", "autonomous.collision", "autonomous.packets", "autonomous.avoided"))

        self.last_packet_count = packets
        self.last_telemetry_seen = time.time()
        self.last_yaw = yaw
        self.last_collision = coll

        self.root.after(0, lambda y=yaw, c=coll, p=packets, a=avoided: self._update_ui_telemetry(y, c, p, a))
        self.root.after(0, lambda y=yaw, c=coll: self._validate_telemetry_values(y, c))
        self.root.after(0, lambda p=packets: self._mark_vision_status("ACTIVE" if p > 0 else "NO PACKETS"))
        self.root.after(0, lambda hp=has_payload: self._mark_camera_state("ACTIVE" if hp else "WAITING"))
        self.root.after(0, lambda: self._append_log(
            f"XIAO telemetry: yaw={yaw:.4f}, collision={coll:.4f}, packets={packets}, avoided={avoided}"))

    def _handle_litewing_control(self, data):
        vx = data.get("ctrltarget.vx", 0.0)
        vy = data.get("ctrltarget.vy", 0.0)
        yaw_rate = data.get("ctrltarget.yaw", 0.0)

        self.root.after(0, lambda y=yaw_rate, fx=vx, fy=vy: self._update_litewing_ui_telemetry(y, fx, fy))
        # Rate limit logging the control targets so we don't spam the debug output too fast
        # (It streams at 10Hz, let's just log it occasionally or not at all to keep UI clean)

    def _update_litewing_ui_telemetry(self, yaw_rate, vx, vy):
        self.litewing_steer_var.set(f"Target Yaw Rate: {yaw_rate:+.2f} deg/s")
        self.litewing_fwd_vel_var.set(f"Target Fwd Vel (vx): {vx:+.2f} m/s")
        self.litewing_ang_vel_var.set(f"Target Lat Vel (vy): {vy:+.2f} m/s")

    def _update_ui_telemetry(self, yaw, coll, packets, avoided):
        self.yaw_var.set(f"Vision Yaw: {yaw:+.4f}")
        self.collision_var.set(f"Collision Prob: {coll:.4f} {'[OBSTACLE!]' if coll > 0.5 else '[CLEAR]'}")
        self.packets_var.set(f"Packets: {packets}")
        self.avoided_var.set(f"Obstacles Avoided: {avoided}")
        self.vision_status_var.set("Vision: ACTIVE" if packets > 0 else "Vision: NO PACKETS")

    def _mark_camera_state(self, state):
        self.cam_status_var.set(f"Camera: {state}")

    def _mark_vision_status(self, state):
        self.vision_status_var.set(f"Vision: {state}")

    def _validate_telemetry_values(self, yaw, collision):
        if yaw < -1.25 or yaw > 1.25:
            self._append_log(f"WARNING: Yaw out of expected range: {yaw:.4f}. Check steering sign / normalization.")
        if collision < -0.05 or collision > 1.05:
            self._append_log(f"WARNING: Collision probability out of expected range: {collision:.4f}. Check sigmoid scaling.")

    def _check_camera_state(self):
        if not self.connected:
            return
        if not self.autonomous_active:
            self._mark_camera_state("OFF")
            self.vision_status_var.set("Vision: OFF")
            return
        if self.last_telemetry_seen is None:
            print("[DEBUG] _check_camera_state: last_telemetry_seen is None! Setting WAITING.")
            self._mark_camera_state("WAITING")
            self.vision_status_var.set("Vision: WAITING")
            self._append_log("No autonomous telemetry received yet; verify the XIAO/LiteWing firmware is running the custom log block")
        elif time.time() - self.last_telemetry_seen > 2.5:
            print("[DEBUG] _check_camera_state: telemetry timed out! Setting NO DATA.")
            self._mark_camera_state("NO DATA")
            self._mark_vision_status("NO DATA")
            self._append_log("Autonomous telemetry timed out; the firmware may not be streaming the expected log variables")
        elif self.last_packet_count == 0:
            print("[DEBUG] _check_camera_state: last_packet_count is 0! Setting NO PACKETS.")
            self._mark_camera_state("NO PACKETS")
            self._mark_vision_status("NO PACKETS")
            self._append_log("Vision log subscription active, but packet count remains 0. Check XIAO firmware and START command reception.")
        elif self.last_yaw is not None and self.last_collision is not None:
            if not (-1.5 <= self.last_yaw <= 1.5) or not (0.0 <= self.last_collision <= 1.2):
                self._append_log(
                    f"Telemetry out of range: yaw={self.last_yaw:.3f}, collision={self.last_collision:.3f}. "
                    "Verify XIAO packet decoding and payload scaling.")
        self.root.after(2500, self._check_camera_state)

    def _append_log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        log_line = f"[{timestamp}] {message}\n"
        try:
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert(tk.END, log_line)
            self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)
            self.debug_status.set(f"Last debug: {time.strftime('%H:%M:%S')}")
        except Exception:
            print(log_line, end='')

    def _clear_debug(self):
        try:
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.delete('1.0', tk.END)
            self.log_text.configure(state=tk.DISABLED)
            self.debug_status.set("Debug cleared")
        except Exception:
            pass

    # ============================================================
    # Controls
    # ============================================================
    def arm_drone(self):
        if not self.connected:
            return
        try:
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            self.cf.commander.send_setpoint(0, 0, 0, 10000)
            self.mode_var.set("Mode: ARMED (Manual)")
        except Exception as e:
            messagebox.showerror("Error", f"Arm failed:\n{e}")

    def disarm_drone(self):
        if not self.connected:
            return
        try:
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            self.mode_var.set("Mode: DISARMED")
            if self.autonomous_active:
                self.stop_autonomous()
        except Exception as e:
            messagebox.showerror("Error", f"Disarm failed:\n{e}")

    def emergency_stop(self):
        if not self.connected:
            return
        try:
            self._set_param("autonomous.enabled", 0)
            self.autonomous_active = False
            self.auto_status_label.config(text="OFF", foreground="gray")
            self.mode_var.set("Mode: EMERGENCY STOP")
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            messagebox.showwarning("Emergency Stop", "Motors cut instantly! Autonomous mode priority released.")
        except Exception as e:
            messagebox.showerror("Error", f"Emergency stop failed:\n{e}")

    def start_autonomous(self):
        if not self.connected:
            return
        if not self.autonomous_param_supported:
            messagebox.showerror(
                "Unsupported Firmware",
                "This Crazyflie firmware does not expose the required autonomous.enabled parameter. "
                "Please update the firmware and try again.")
            return
        if self._set_param("autonomous.enabled", 1):
            self.autonomous_active = True
            self.auto_status_label.config(text="ON", foreground="green")
            self.mode_var.set("Mode: AUTONOMOUS (Dynamic Vision)")
            self.last_telemetry_seen = None
            self._mark_camera_state("STARTING")
            self.root.after(2500, self._check_camera_state)
            threading.Thread(target=self._verify_autonomous_param, daemon=True).start()

    def stop_autonomous(self):
        if not self.connected:
            return
        if self._set_param("autonomous.enabled", 0):
            self.autonomous_active = False
            self.auto_status_label.config(text="OFF", foreground="gray")
            self.mode_var.set("Mode: MANUAL")
            self._mark_camera_state("OFF")

    def _set_param(self, name, value):
        if self.cf:
            try:
                self.cf.param.set_value(name, value)
                self._append_log(f"Parameter set: {name} = {value}")
                return True
            except Exception as e:
                self._append_log(f"Parameter write failed: {name} = {value} -> {e}")
                messagebox.showerror("Parameter Write Failed", f"Could not set {name} to {value}:\n{e}")
                return False
        self._append_log(f"Parameter write skipped; no connection: {name}")
        return False

    def _keep_alive_loop(self):
        while self.connected and self.cf:
            try:
                # Harmless ping to keep the ESP32 WiFi watchdog alive
                # even if the drone's CRTP txQueue is starved.
                self.cf.param.request_param_update("autonomous.enabled")
            except Exception:
                pass
            time.sleep(0.2)

    def _on_connected(self, link_uri):
        self.connected = True
        self.status_label.config(text="Status: Connected", foreground="green")
        self.connect_btn.config(text="Disconnect")
        self.arm_btn.config(state=tk.NORMAL)
        self.disarm_btn.config(state=tk.NORMAL)
        self.start_auto_btn.config(state=tk.NORMAL)
        self.stop_auto_btn.config(state=tk.NORMAL)
        self.emerg_btn.config(state=tk.NORMAL)
        self._append_log(f"Connected to {link_uri}")
        threading.Thread(target=self._probe_autonomous_support, daemon=True).start()
        threading.Thread(target=self._keep_alive_loop, daemon=True).start()
        self._setup_log_block()

    def _on_connection_failed(self, link_uri, message):
        self.status_label.config(text="Status: Connection failed", foreground="red")
        self._append_log(f"Connection failed: {link_uri} -> {message}")
        messagebox.showerror("Error", f"Could not connect to {link_uri}:\n{message}")
        self.cf = None

    def _on_disconnected(self, link_uri):
        self._append_log(f"Disconnected from {link_uri}")
        self.disconnect()

    def _probe_autonomous_support(self):
        if not self.cf:
            return
        try:
            value = self.cf.param.get_value("autonomous.enabled")
            self.autonomous_param_supported = True
            self.root.after(0, lambda: self._set_firmware_ok(value))
        except Exception as e:
            self.autonomous_param_supported = False
            self.root.after(0, lambda: self._set_firmware_missing(e))

    def _verify_autonomous_param(self):
        if not self.cf:
            return
        try:
            value = self.cf.param.get_value("autonomous.enabled")
            if not self.autonomous_active and value == 1:
                self.root.after(0, lambda: self._append_log(
                    "Warning: autonomous.enabled is 1 but autonomous mode is not active on the GUI."))
            self.root.after(0, lambda: self._append_log(f"Verified autonomous.enabled = {value}"))
        except Exception as e:
            self.root.after(0, lambda: self._append_log(f"Verification failed: autonomous.enabled read-back failed: {e}"))

    def _set_firmware_ok(self, value):
        self.firmware_var.set("Firmware: OK")
        self.autonomous_param_supported = True
        self._append_log(f"Autonomous parameter detected: autonomous.enabled={value}")

    def _set_firmware_missing(self, error):
        self.firmware_var.set("Firmware: MISSING")
        self.autonomous_param_supported = False
        self.start_auto_btn.config(state=tk.DISABLED)
        self.stop_auto_btn.config(state=tk.DISABLED)
        self._mark_camera_state("NOT SUPPORTED")
        self._append_log(f"Autonomous firmware probe failed: {error}")


if __name__ == "__main__":
    root = tk.Tk()
    app = LiteWingGUI(root)
    root.mainloop()
