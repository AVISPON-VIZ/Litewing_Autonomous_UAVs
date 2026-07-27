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
        self.log_config = None
        self.connected = False
        self.autonomous_active = False

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
        self.cam_status_var = tk.StringVar(value="Camera: OFF")
        self.yaw_var = tk.StringVar(value="Vision Yaw: --")
        self.collision_var = tk.StringVar(value="Collision Prob: --")
        self.packets_var = tk.StringVar(value="Packets: 0")
        self.avoided_var = tk.StringVar(value="Obstacles Avoided: 0")

        ttk.Label(status_frame, textvariable=self.battery_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.mode_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.cam_status_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.yaw_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.collision_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.packets_var).pack(anchor=tk.W)
        ttk.Label(status_frame, textvariable=self.avoided_var).pack(anchor=tk.W)

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
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky=tk.NS)
        self.log_text.config(yscrollcommand=scrollbar.set)

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
        except Exception as e:
            self.status_label.config(text=f"Status: Error - {e}", foreground="red")
            messagebox.showerror("Error", f"Connection failed:\n{e}")

    def disconnect(self):
        if self.cf:
            if self.log_config:
                try:
                    self.log_config.stop()
                except Exception:
                    pass
            try:
                self.cf.close_link()
            except Exception:
                pass
            self.cf = None
        self.connected = False
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.cam_status_var.set("Camera: OFF")
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
            if self.log_config:
                try:
                    self.log_config.stop()
                except Exception:
                    pass
                self.log_config = None

            self.log_config = LogConfig(name="AutonomousNav", period_in_ms=100)
            self.log_config.add_variable("autonomous.yaw", "float")
            self.log_config.add_variable("autonomous.collision", "float")
            self.log_config.add_variable("autonomous.packets", "uint32_t")
            self.log_config.add_variable("autonomous.avoided", "uint32_t")

            self.cf.log.add_config(self.log_config)
            self.log_config.data_received_cb.add_callback(self._log_data_received)
            self.log_config.start()
            self._append_log("Log subscription started for autonomous telemetry")
        except Exception as e:
            self._append_log(f"Log block subscription warning: {e}")
            self.status_label.config(text="Status: Log subscription failed", foreground="red")

    def _log_data_received(self, timestamp, data, logconf):
        yaw = data.get("autonomous.yaw", 0.0)
        coll = data.get("autonomous.collision", 0.0)
        packets = data.get("autonomous.packets", 0)
        avoided = data.get("autonomous.avoided", 0)

        self.root.after(0, lambda: self._update_ui_telemetry(yaw, coll, packets, avoided))
        self.root.after(0, lambda: self._append_log(
            f"Telemetry received: yaw={yaw:.4f}, collision={coll:.4f}, packets={packets}, avoided={avoided}"))
        self.root.after(0, lambda: self.cam_status_var.set("Camera: ACTIVE"))

    def _update_ui_telemetry(self, yaw, coll, packets, avoided):
        self.yaw_var.set(f"Vision Yaw: {yaw:+.4f}")
        self.collision_var.set(f"Collision Prob: {coll:.4f} {'[OBSTACLE!]' if coll > 0.5 else '[CLEAR]'}")
        self.packets_var.set(f"Packets: {packets}")
        self.avoided_var.set(f"Obstacles Avoided: {avoided}")

    def _append_log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        log_line = f"[{timestamp}] {message}\n"
        try:
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert(tk.END, log_line)
            self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)
        except Exception:
            print(log_line, end='')

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
        if self._set_param("autonomous.enabled", 1):
            self.autonomous_active = True
            self.auto_status_label.config(text="ON", foreground="green")
            self.mode_var.set("Mode: AUTONOMOUS (Dynamic Vision)")
            self.cam_status_var.set("Camera: ACTIVE")

    def stop_autonomous(self):
        if not self.connected:
            return
        if self._set_param("autonomous.enabled", 0):
            self.autonomous_active = False
            self.auto_status_label.config(text="OFF", foreground="gray")
            self.mode_var.set("Mode: MANUAL")
            self.cam_status_var.set("Camera: OFF")

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
        self._setup_log_block()

    def _on_connection_failed(self, link_uri, message):
        self.status_label.config(text="Status: Connection failed", foreground="red")
        self._append_log(f"Connection failed: {link_uri} -> {message}")
        messagebox.showerror("Error", f"Could not connect to {link_uri}:\n{message}")
        self.cf = None

    def _on_disconnected(self, link_uri):
        self._append_log(f"Disconnected from {link_uri}")
        self.disconnect()


if __name__ == "__main__":
    root = tk.Tk()
    app = LiteWingGUI(root)
    root.mainloop()
