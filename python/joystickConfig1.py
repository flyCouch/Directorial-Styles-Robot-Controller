import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pygame
import json
import os
import threading
import time

# --- Configuration Constants ---
CONFIG_FILE = "joystick_config.json"
ACTIONS = [
    "X-Axis (analog)",
    "Y-Axis (analog)",
    "R-Rotation (analog)",
    "Elevation (analog)",
    "Laser On/Off (toggle)",
    "Power (toggle)",
    "Speed Up (button)",
    "Speed Down (button)",
    # Add more actions as needed
]
ANALOG_AXIS_THRESHOLD = 0.1 # Minimum movement for an analog axis to register
ANALOG_DISPLAY_PRECISION = 2 # Decimal places for analog values

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
        # print(f"Detected {self.num_joysticks} joysticks.") # Commented out for cleaner console
        # for i, joy in enumerate(self.joysticks): # Commented out for cleaner console
        #     print(f"  Joystick {i}: {joy.get_name()}")

    def set_active_joystick(self, index):
        if 0 <= index < len(self.joysticks):
            self.active_joystick = self.joysticks[index]
            # print(f"Active joystick set to: {self.active_joystick.get_name()}") # Commented out
            return True
        return False

    def get_joystick_names(self):
        return [joy.get_name() for joy in self.joysticks]

    def poll_events(self):
        events = []
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                events.append({'type': 'button_down', 'joy_id': event.joy, 'button': event.button})
            elif event.type == pygame.JOYBUTTONUP:
                events.append({'type': 'button_up', 'joy_id': event.joy, 'button': event.button})
            elif event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > ANALOG_AXIS_THRESHOLD:
                    events.append({'type': 'axis_motion', 'joy_id': event.joy, 'axis': event.axis, 'value': event.value})
            elif event.type == pygame.JOYHATMOTION:
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
        # Increased initial window size
        master.geometry("900x700") # Wider to accommodate buttons, taller for more rows

        self.joystick_manager = JoystickManager()
        self.current_config = {} # {'action_name': {'joy_name': '...', 'input_type': '...', 'input_id': '...'}}
        self.mapping_widgets = {} # Stores references to labels/buttons for each row
        self.assignment_mode = False
        self.current_action_to_assign = None

        self._setup_ui()
        self._load_config() # Attempt to load config on startup
        self._update_joystick_selection_ui()
        self._start_polling()

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
        # Only process events if an active joystick is selected
        if self.joystick_manager.active_joystick:
            events = self.joystick_manager.poll_events() # Get all events in this cycle

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
                            # No 'return' here. Allow other events in this cycle to be processed.
                            break # Stop processing events for assignment, as we found one
            
            # Always update indicator lights for current state, regardless of assignment mode
            current_state = self.joystick_manager.get_current_state()
            for action_name, widgets in self.mapping_widgets.items():
                config_map = self.current_config.get(action_name)
                indicator_canvas = widgets['indicator_light']
                # Only update if current joystick is active and mapping is for this joystick
                if config_map and \
                   self.joystick_manager.active_joystick and \
                   config_map.get('joy_name') == self.joystick_manager.active_joystick.get_name():
                    
                    input_type = config_map['input_type']
                    input_id = config_map['input_id']
                    is_active = False
                    
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
                            # Only update if the text is currently showing a dynamic axis value
                            if "Axis" in widgets['mapped_label'].cget("text") and "(" in widgets['mapped_label'].cget("text"):
                                widgets['mapped_label'].config(text=f"Axis {input_id} (Analog)")
                            
                    elif input_type == 'hat' and input_id in current_state['hats']:
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
                if mapping.get('joy_name') == self.joystick_manager.active_joystick.get_name():
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

    def on_closing(self):
        print("Closing application.")
        self.joystick_manager.quit()
        self.master.destroy()

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    app = JoystickConfiguratorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing) # Handle window close event
    root.mainloop()
