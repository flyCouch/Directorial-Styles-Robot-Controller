import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pygame
import json
import os
import threading
import time
import socket # Import socket for network communication

# --- Configuration Constants ---
CONFIG_FILE = "joystick_config.json"
ACTIONS = [
    "X-Axis (analog)",
    "Y-Axis (analog)",
    "R-Rotation (analog)",
    "E-Elevation (analog)",
    "Laser On/Off (toggle)",
    "Power (toggle)",
    "Speed Up (button)",
    "Speed Down (button)",
    # Add more actions as needed
]
ANALOG_AXIS_THRESHOLD = 0.01 # Minimum movement for an analog axis to register
ANALOG_DISPLAY_PRECISION = 2 # Decimal places for analog values

# --- Network Configuration ---
HOST = '127.0.0.1'  # Listen on localhost. Use '0.0.0.0' to listen on all available interfaces.
PORT = 12345        # Port to listen on. Choose an available port.

# --- Joystick Manager Class ---
class JoystickManager:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joysticks = []
        self.active_joystick = None
        self.num_joysticks = 0
        self.detect_joysticks()

    def detect_joysticks(self):
        pygame.joystick.quit() # Re-initialize to detect new joysticks
        pygame.joystick.init()
        self.num_joysticks = pygame.joystick.get_count()
        self.joysticks = []
        for i in range(self.num_joysticks):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            self.joysticks.append(joy)

    def set_active_joystick(self, index):
        if 0 <= index < len(self.joysticks):
            self.active_joystick = self.joysticks[index]
            return True
        return False

    def get_joystick_names(self):
        return [joy.get_name() for joy in self.joysticks]

    def poll_events(self):
        # Crucial for Pygame to process its internal event queue
        pygame.event.pump()
        events = []
        # Get all new events since the last call
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                events.append({'type': 'button_down', 'joy_id': event.joy, 'button': event.button})
            elif event.type == pygame.JOYBUTTONUP:
                events.append({'type': 'button_up', 'joy_id': event.joy, 'button': event.button})
            elif event.type == pygame.JOYAXISMOTION:
                # Only add axis motion event if it's beyond the threshold
                if abs(event.value) > ANALOG_AXIS_THRESHOLD:
                    events.append({'type': 'axis_motion', 'joy_id': event.joy, 'axis': event.axis, 'value': event.value})
            elif event.type == pygame.JOYHATMOTION:
                # Only add hat motion event if it's not at (0,0) (neutral position)
                if event.value != (0,0):
                    events.append({'type': 'hat_motion', 'joy_id': event.joy, 'hat': event.hat, 'value': event.value})
        return events

    def get_current_state(self):
        state = {'buttons': {}, 'axes': {}, 'hats': {}}
        if self.active_joystick:
            for i in range(self.active_joystick.get_numbuttons()):
                state['buttons'][i] = self.active_joystick.get_button(i)
            for i in range(self.active_joystick.get_numaxes()):
                state['axes'][i] = self.active_joystick.get_axis(i)
            for i in range(self.active_joystick.get_numhats()):
                state['hats'][i] = self.active_joystick.get_hat(i)
        return state

    def quit(self):
        pygame.joystick.quit()
        pygame.quit()

# --- GUI Class ---
class JoystickConfiguratorApp:
    def __init__(self, master):
        self.master = master
        master.title("Robot Joystick Configurator")
        master.geometry("900x700")

        self.joystick_manager = JoystickManager()
        self.current_config = {} # {'action_name': {'joy_name': '...', 'input_type': '...', 'input_id': '...'}}
        self.mapping_widgets = {} # Stores references to labels/buttons for each row
        self.assignment_mode = False
        self.current_action_to_assign = None

        # --- Socket Variables ---
        self.server_socket = None
        self.client_connection = None
        self.client_address = None
        self.server_thread = None
        self.running_server = True # Flag to control server thread

        # --- New variables for JSON payload ---
        self.current_speed = 0.5 # Initial speed value, can be adjusted (e.g., 0.0 to 1.0)
        self.last_sent_payload = {} # To avoid sending redundant data

        self._setup_ui()
        self._load_config() # Attempt to load config on startup
        self._update_joystick_selection_ui()
        self._setup_socket_server() # Setup the socket server
        self._start_polling() # Start polling joystick and sending data

    def _setup_ui(self):
        # --- Top Frame: Joystick Selection & Controls ---
        top_frame = ttk.Frame(self.master, padding="10")
        top_frame.pack(fill=tk.X)

        # Left side of top_frame
        left_top_frame = ttk.Frame(top_frame)
        left_top_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(left_top_frame, text="Select Joystick:").pack(side=tk.LEFT, padx=5)
        self.joystick_var = tk.StringVar(self.master)
        self.joystick_dropdown = ttk.Combobox(left_top_frame, textvariable=self.joystick_var, state="readonly", width=30)
        self.joystick_dropdown.pack(side=tk.LEFT, padx=5)
        self.joystick_dropdown.bind("<<ComboboxSelected>>", self._on_joystick_selected)
        ttk.Button(left_top_frame, text="Rescan Joysticks", command=self._rescan_joysticks).pack(side=tk.LEFT, padx=10)

        # Right side of top_frame (for Save/Load buttons)
        right_top_frame = ttk.Frame(top_frame)
        right_top_frame.pack(side=tk.RIGHT, padx=5) # pack to the right
        ttk.Button(right_top_frame, text="Load Config", command=self._load_config).pack(side=tk.RIGHT, padx=5)
        ttk.Button(right_top_frame, text="Save Config", command=self._save_config).pack(side=tk.RIGHT, padx=5)

        # --- Main Frame: Mappings Table ---
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Headers
        ttk.Label(main_frame, text="Action", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(main_frame, text="Mapped Input", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Label(main_frame, text="Live Indicator", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Label(main_frame, text="Assign", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=3, padx=5, pady=5, sticky="w")

        # Grid configuration for dynamic rows
        self.action_rows_frame = ttk.Frame(main_frame)
        self.action_rows_frame.grid(row=1, column=0, columnspan=4, sticky="nsew")
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)
        main_frame.grid_columnconfigure(2, weight=0)
        main_frame.grid_columnconfigure(3, weight=0) # Assign button column doesn't expand

        self._populate_mapping_rows()

    def _populate_mapping_rows(self):
        # Clear existing rows
        for widget in self.action_rows_frame.winfo_children():
            widget.destroy()
        self.mapping_widgets = {}

        for i, action_name in enumerate(ACTIONS):
            row_num = i

            # Action Label
            ttk.Label(self.action_rows_frame, text=action_name).grid(row=row_num, column=0, padx=5, pady=2, sticky="w")

            # Mapped Input Label
            mapped_input_label = ttk.Label(self.action_rows_frame, text="Not assigned", relief="groove") # Added relief for clarity
            mapped_input_label.grid(row=row_num, column=1, padx=5, pady=2, sticky="ew") # Changed to sticky="ew" for expansion

            # Indicator Light
            indicator_light = tk.Canvas(self.action_rows_frame, width=20, height=20, bg="gray", relief="ridge")
            indicator_light.grid(row=row_num, column=2, padx=5, pady=2)
            indicator_light.create_oval(2, 2, 18, 18, fill="darkgray", outline="black", tags="light_oval")

            # Assign Button
            assign_button = ttk.Button(self.action_rows_frame, text="Assign",
                                       command=lambda a=action_name: self._start_assignment(a))
            assign_button.grid(row=row_num, column=3, padx=5, pady=2)

            self.mapping_widgets[action_name] = {
                'mapped_label': mapped_input_label,
                'indicator_light': indicator_light,
                'assign_button': assign_button
            }
            self.action_rows_frame.grid_columnconfigure(0, weight=1)
            self.action_rows_frame.grid_columnconfigure(1, weight=1)
            self.action_rows_frame.grid_columnconfigure(2, weight=0)
            self.action_rows_frame.grid_columnconfigure(3, weight=0)

    def _update_joystick_selection_ui(self):
        names = self.joystick_manager.get_joystick_names()
        if not names:
            self.joystick_dropdown['values'] = ["No Joysticks Found"]
            self.joystick_var.set("No Joysticks Found")
            self.joystick_dropdown.config(state="disabled")
            self.joystick_manager.active_joystick = None
        else:
            self.joystick_dropdown['values'] = names
            self.joystick_dropdown.config(state="readonly")
            if not self.joystick_manager.active_joystick or \
               self.joystick_manager.active_joystick.get_name() not in names:
                # Select the first joystick if no active one or active one is gone
                self.joystick_var.set(names[0])
                self.joystick_manager.set_active_joystick(0)
            else:
                # Keep current selection if joystick is still connected
                current_name = self.joystick_manager.active_joystick.get_name()
                if current_name in names:
                    self.joystick_var.set(current_name)
                else:
                    self.joystick_var.set(names[0])
                    self.joystick_manager.set_active_joystick(0)

        self._update_mapped_inputs_display()

    def _on_joystick_selected(self, event=None):
        selected_name = self.joystick_var.get()
        if selected_name == "No Joysticks Found":
            self.joystick_manager.active_joystick = None
            return

        for i, name in enumerate(self.joystick_manager.get_joystick_names()):
            if name == selected_name:
                self.joystick_manager.set_active_joystick(i)
                break
        self._update_mapped_inputs_display()

    def _rescan_joysticks(self):
        self.joystick_manager.detect_joysticks()
        self._update_joystick_selection_ui()

    def _start_assignment(self, action_name):
        if not self.joystick_manager.active_joystick:
            messagebox.showwarning("No Joystick Selected", "Please select an active joystick first.")
            return

        self.assignment_mode = True
        self.current_action_to_assign = action_name
        self.master.title(f"Robot Joystick Configurator - Assigning: {action_name}")
        self.mapping_widgets[action_name]['mapped_label'].config(text="PRESS INPUT...", background="yellow") # Highlight current action
        self._set_assign_buttons_state("disabled") # Disable all assign buttons during assignment

    def _end_assignment(self):
        """Resets assignment mode and updates UI after assignment."""
        if self.current_action_to_assign:
            # Reset background to default system button face color
            self.mapping_widgets[self.current_action_to_assign]['mapped_label'].config(background="")
        self.assignment_mode = False
        self.current_action_to_assign = None
        self.master.title("Robot Joystick Configurator")
        self._set_assign_buttons_state("normal") # Re-enable all assign buttons

    def _set_assign_buttons_state(self, state):
        """Enables or disables all 'Assign' buttons."""
        for action_name in ACTIONS:
            if action_name in self.mapping_widgets:
                self.mapping_widgets[action_name]['assign_button'].config(state=state)

    def _handle_joystick_events(self):
        # Crucial for Pygame to process its internal event queue
        pygame.event.pump()

        events = []
        # Get all new events since the last call
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                events.append({'type': 'button_down', 'joy_id': event.joy, 'button': event.button})
            elif event.type == pygame.JOYBUTTONUP:
                events.append({'type': 'button_up', 'joy_id': event.joy, 'button': event.button})
            elif event.type == pygame.JOYAXISMOTION:
                # Only add axis motion event if it's beyond the threshold
                if abs(event.value) > ANALOG_AXIS_THRESHOLD:
                    events.append({'type': 'axis_motion', 'joy_id': event.joy, 'axis': event.axis, 'value': event.value})
            elif event.type == pygame.JOYHATMOTION:
                # Only add hat motion event if it's not at (0,0) (neutral position)
                if event.value != (0,0):
                    events.append({'type': 'hat_motion', 'joy_id': event.joy, 'hat': event.hat, 'value': event.value})

        assigned_this_cycle = False
        if self.assignment_mode and self.current_action_to_assign:
            for event in events:
                if event['joy_id'] == self.joystick_manager.active_joystick.get_id():
                    mapped_info = {
                        'joy_name': self.joystick_manager.active_joystick.get_name()
                    }
                    
                    if event['type'] == 'button_down':
                        mapped_info.update({'input_type': 'button', 'input_id': event['button']})
                        assigned_this_cycle = True
                    elif event['type'] == 'axis_motion':
                        mapped_info.update({'input_type': 'axis', 'input_id': event['axis']})
                        assigned_this_cycle = True
                    elif event['type'] == 'hat_motion':
                        mapped_info.update({'input_type': 'hat', 'input_id': event['hat'], 'value': event['value']})
                        assigned_this_cycle = True

                    if assigned_this_cycle:
                        self.current_config[self.current_action_to_assign] = mapped_info
                        self._update_mapped_inputs_display() # Update display with new mapping
                        self._end_assignment() # Crucial: Reset assignment mode
                        break # Only assign one input per assignment cycle

        # --- Construct and Send JSON Payload based on Current State ---
        if self.joystick_manager.active_joystick and not self.assignment_mode: # Only send data if not in assignment mode
            current_state = self.joystick_manager.get_current_state()
            
            # Initialize command payload with default values
            command_payload = {
                "x": 0.0,
                "y": 0.0,
                "r": 0.0,
                "e": 0.0,
                "laser": 0,
                "power": 0,
                "speed": self.current_speed # Use the internal speed state
            }

            for action_name in ACTIONS:
                config_map = self.current_config.get(action_name)

                if config_map and config_map.get('joy_name') == self.joystick_manager.active_joystick.get_name():
                    input_type = config_map['input_type']
                    input_id = config_map['input_id']

                    if action_name == "X-Axis (analog)" and input_type == 'axis' and input_id in current_state['axes']:
                        command_payload["x"] = round(current_state['axes'][input_id], ANALOG_DISPLAY_PRECISION)
                    elif action_name == "Y-Axis (analog)" and input_type == 'axis' and input_id in current_state['axes']:
                        command_payload["y"] = round(current_state['axes'][input_id], ANALOG_DISPLAY_PRECISION)
                    elif action_name == "R-Rotation (analog)" and input_type == 'axis' and input_id in current_state['axes']:
                        command_payload["r"] = round(current_state['axes'][input_id], ANALOG_DISPLAY_PRECISION)
                    #elif action_name == "E-Elevation (analog)" and input_type == 'axis' and input_id in current_state['axes']:
                        #command_payload["e"] = round(current_state['axes'][input_id], ANALOG_DISPLAY_PRECISION)
                    elif action_name == "E-Elevation (analog)" and input_type == 'axis' and input_id in current_state['axes']:
                        command_payload["e"] = round(current_state['axes'][input_id], ANALOG_DISPLAY_PRECISION)
                    elif action_name == "Laser On/Off (toggle)" and input_type == 'button' and input_id in current_state['buttons']:
                        command_payload["laser"] = current_state['buttons'][input_id] # 1 if pressed, 0 if not
                    elif action_name == "Power (toggle)" and input_type == 'button' and input_id in current_state['buttons']:
                        command_payload["power"] = current_state['buttons'][input_id] # 1 if pressed, 0 if not
                    elif action_name == "Speed Up (button)" and input_type == 'button' and input_id in current_state['buttons']:
                        # Only increment speed on button *press* (JOYBUTTONDOWN event)
                        # Iterate through captured events for the *down* press
                        for event in events:
                            if event['type'] == 'button_down' and event['button'] == input_id:
                                self.current_speed = min(1.0, self.current_speed + 0.1) # Increment speed, cap at 1.0
                                command_payload["speed"] = self.current_speed
                                break # Process only one press per cycle for this button
                    elif action_name == "Speed Down (button)" and input_type == 'button' and input_id in current_state['buttons']:
                        # Only decrement speed on button *press* (JOYBUTTONDOWN event)
                        for event in events:
                            if event['type'] == 'button_down' and event['button'] == input_id:
                                self.current_speed = max(0.0, self.current_speed - 0.1) # Decrement speed, cap at 0.0
                                command_payload["speed"] = self.current_speed
                                break # Process only one press per cycle for this button

            # Send the JSON payload if it's different from the last one sent
            if command_payload != self.last_sent_payload:
                self._send_joystick_data(json.dumps(command_payload))
                self.last_sent_payload = command_payload.copy() # Store a copy to compare in next cycle

        # --- Always update indicator lights and axis values on the GUI ---
        current_state = self.joystick_manager.get_current_state()
        for action_name, widgets in self.mapping_widgets.items():
            config_map = self.current_config.get(action_name)
            indicator_canvas = widgets['indicator_light']
            
            is_active = False

            # Only update if current joystick is active and mapping is for this joystick
            if config_map and \
               self.joystick_manager.active_joystick and \
               config_map.get('joy_name') == self.joystick_manager.active_joystick.get_name():
                
                input_type = config_map['input_type']
                input_id = config_map['input_id']
                
                if input_type == 'button' and input_id in current_state['buttons']:
                    if current_state['buttons'][input_id] == 1: # Button is pressed
                        is_active = True
                elif input_type == 'axis' and input_id in current_state['axes']:
                    axis_value = current_state['axes'][input_id]
                    if abs(axis_value) > ANALOG_AXIS_THRESHOLD: # Axis moved beyond threshold
                        is_active = True
                        widgets['mapped_label'].config(text=f"Axis {input_id} ({axis_value:.{ANALOG_DISPLAY_PRECISION}f})")
                    else:
                        # If not active, but was showing a value, reset to base analog text
                        if "Axis" in widgets['mapped_label'].cget("text") and "(" in widgets['mapped_label'].cget("text"):
                            widgets['mapped_label'].config(text=f"Axis {input_id} (Analog)")
                            
                elif input_type == 'hat' and input_id in current_state['hats']:
                    # Check if the current hat state matches the configured hat value for this action
                    if current_state['hats'][input_id] == config_map.get('value'):
                        is_active = True

                light_color = "limegreen" if is_active else "darkgray"
                indicator_canvas.itemconfig("light_oval", fill=light_color)
            else:
                # If not mapped to active joystick or no joystick selected, ensure lights are off
                indicator_canvas.itemconfig("light_oval", fill="darkgray")

        # Re-schedule polling
        self.master.after(50, self._handle_joystick_events)

    def _start_polling(self):
        # Initial call to start the event loop
        self.master.after(50, self._handle_joystick_events)

    def _update_mapped_inputs_display(self):
        """Updates the 'Mapped Input' labels based on `self.current_config` and active joystick."""
        for action_name, widgets in self.mapping_widgets.items():
            config_map = self.current_config.get(action_name)
            
            # Reset visual state first
            widgets['mapped_label'].config(text="Not assigned", background="") # Reset background to default
            widgets['indicator_light'].itemconfig("light_oval", fill="darkgray")
            
            # If there's a config for this action AND it's for the currently active joystick
            if config_map and \
               self.joystick_manager.active_joystick and \
               config_map.get('joy_name') == self.joystick_manager.active_joystick.get_name():
                
                input_type = config_map['input_type']
                input_id = config_map['input_id']
                display_text = "Unknown Input"

                if input_type == 'button':
                    display_text = f"Button {input_id}"
                elif input_type == 'axis':
                    display_text = f"Axis {input_id} (Analog)"
                elif input_type == 'hat':
                    display_text = f"Hat {input_id} {config_map.get('value', '')}"
                
                widgets['mapped_label'].config(text=display_text)
            else:
                # If mapped to another joystick or not mapped to anything, ensure it shows "Not assigned"
                widgets['mapped_label'].config(text="Not assigned")
                widgets['indicator_light'].itemconfig("light_oval", fill="darkgray")

    def _save_config(self):
        if not self.joystick_manager.active_joystick:
            messagebox.showwarning("Save Error", "No joystick selected to save configuration for.")
            return

        file_path = filedialog.asksaveasfilename(defaultextension=".json",
                                                 filetypes=[("JSON files", "*.json")],
                                                 initialfile=CONFIG_FILE)
        if file_path:
            # Only save mappings for the active joystick for simplicity
            config_to_save = {}
            for action, mapping in self.current_config.items():
                # Ensure we only save mappings that are relevant to the selected joystick
                # Or, if no joystick is selected, save all current mappings (though typically
                # config is per-joystick name)
                if not self.joystick_manager.active_joystick or \
                   mapping.get('joy_name') == self.joystick_manager.active_joystick.get_name():
                    config_to_save[action] = mapping

            try:
                with open(file_path, 'w') as f:
                    json.dump(config_to_save, f, indent=4)
                messagebox.showinfo("Save Config", f"Configuration saved to {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("Save Error", f"Failed to save configuration: {e}")

    def _load_config(self):
        file_path = filedialog.askopenfilename(defaultextension=".json",
                                               filetypes=[("JSON files", "*.json")],
                                               initialfile=CONFIG_FILE)
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    loaded_config = json.load(f)
                
                # Clear current config and load new one
                self.current_config = loaded_config
                
                # If an active joystick is selected, update its display immediately
                if self.joystick_manager.active_joystick:
                    self._update_mapped_inputs_display()
                    messagebox.showinfo("Load Config", f"Configuration loaded from {os.path.basename(file_path)}")
                else:
                    messagebox.showinfo("Load Config", f"Configuration loaded from {os.path.basename(file_path)}. Select a joystick to apply.")
                
            except FileNotFoundError:
                messagebox.showerror("Load Error", "Configuration file not found.")
            except json.JSONDecodeError:
                messagebox.showerror("Load Error", "Invalid JSON format in configuration file.")
            except Exception as e:
                messagebox.showerror("Load Error", f"An error occurred: {e}")

    def _setup_socket_server(self):
        """Sets up the TCP socket server to listen for client connections."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow reuse of address
            self.server_socket.bind((HOST, PORT))
            self.server_socket.listen(1) # Listen for one incoming connection at a time
            print(f"Socket server listening on {HOST}:{PORT}")

            # Start a separate thread to accept connections to avoid blocking GUI
            self.server_thread = threading.Thread(target=self._accept_connections_loop, daemon=True)
            self.server_thread.start()
        except Exception as e:
            print(f"Error setting up socket server: {e}")
            messagebox.showerror("Socket Error", f"Failed to set up socket server: {e}")

    def _accept_connections_loop(self):
        """Loop to accept incoming client connections."""
        while self.running_server:
            try:
                if self.client_connection is None: # Only accept if no client is currently connected
                    print("Waiting for client connection...")
                    # This accept call will block until a client connects
                    self.server_socket.settimeout(1.0) # Set a timeout so thread can check running_server flag
                    conn, addr = self.server_socket.accept()
                    self.client_connection = conn
                    self.client_address = addr
                    print(f"Accepted connection from {addr}")
                    self.server_socket.settimeout(None) # Remove timeout once connected
                else:
                    time.sleep(0.1) # Small delay when client is already connected
            except socket.timeout:
                pass # No connection within timeout, continue loop
            except Exception as e:
                if self.running_server: # Only print error if server is expected to be running
                    print(f"Error in accept connections loop: {e}")
                if self.client_connection:
                    self.client_connection.close()
                self.client_connection = None
                self.client_address = None
            finally:
                pygame.event.pump() # Ensure pygame events are processed even in this thread

    def _send_joystick_data(self, message_string):
        """Sends the joystick event string over the socket if a client is connected."""
        
        print(f"Sent: {message_string.strip()}")      

        if self.client_connection:
            try:
                # Add newline delimiter for client parsing
                # Encode the string as bytes before sending
                self.client_connection.sendall((message_string + '\n').encode('utf-8'))
                # print(f"Sent: {message_string.strip()}") # Uncomment for debugging sent data
            except (BrokenPipeError, ConnectionResetError) as e:
                print(f"Client disconnected: {e}")
                if self.client_connection:
                    self.client_connection.close()
                self.client_connection = None
                self.client_address = None
            except Exception as e:
                print(f"Error sending data: {e}")
                if self.client_connection:
                    self.client_connection.close()
                self.client_connection = None
                self.client_address = None

    def on_closing(self):
        print("Closing application.")
        self.running_server = False # Signal the server thread to stop
        
        # Give the server thread a moment to notice the flag and unblock accept()
        if self.server_thread and self.server_thread.is_alive():
            # If server_socket.accept() is blocking, closing the socket will raise an exception
            # in the server thread, causing it to exit the loop.
            try:
                self.server_socket.shutdown(socket.SHUT_RDWR)
            except OSError as e:
                # Handle cases where socket is already closed or in a bad state
                if "socket is not connected" not in str(e) and "Bad file descriptor" not in str(e):
                    print(f"Warning during server socket shutdown: {e}")
            self.server_thread.join(timeout=1.0) # Give thread a moment to finish

        if self.client_connection:
            try:
                self.client_connection.shutdown(socket.SHUT_RDWR)
                self.client_connection.close()
            except OSError as e:
                print(f"Error during client connection shutdown/close: {e}")

        if self.server_socket:
            try:
                self.server_socket.close()
            except OSError as e:
                print(f"Error during server socket close: {e}")

        self.joystick_manager.quit()
        self.master.destroy()

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    app = JoystickConfiguratorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing) # Handle window close event
    root.mainloop()
