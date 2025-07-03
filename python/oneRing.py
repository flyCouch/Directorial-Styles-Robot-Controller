import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import sys

class ScriptManagerApp:
    def __init__(self, master):
        self.master = master
        master.title("OneRing - Python Script Launcher")
        master.geometry("800x600") # Increased size for more scripts

        self.running_scripts = {} # Stores {script_name: {'process': Popen_object, 'label_widget': tk.Label, 'button_widget': ttk.Button}}

        # --- SCRIPT CONFIGURATION ---
        # IMPORTANT: Configure your scripts here!
        # Each entry should be a dictionary with:
        # 'name': Display name for the script button.
        # 'path': Full path to the Python script (.py file).
        # 'venv_path': Full path to the root of its virtual environment (e.g., 'path/to/my_project/venv').

        # Examples (MODIFY THESE TO YOUR ACTUAL PATHS):
        self.scripts_config = [
            {
                'name': "Joystick Configurator",
                'path': "/home/ron/Python/3wheelerAccessories/joystickConfig1.py",
                'venv_path': "/home/ron/Python/3wheelerAccessories/venv"
            },
            {
                'name': "Motor Control Script",
                'path': "/home/ron/Python/MotorControl/motor_control.py",
                'venv_path': "/home/ron/Python/MotorControl/venv"
            },
            {
                'name': "Sensor Data Logger",
                'path': "/home/ron/Python/SensorLog/sensor_logger.py",
                'venv_path': "/home/ron/Python/SensorLog/venv_env" # Example of different venv name
            },
            {
                'name': "Web Server API",
                'path': "/home/ron/Python/WebServer/api_server.py",
                'venv_path': "/home/ron/Python/WebServer/env"
            },
            {
                'name': "Camera Feed Processor",
                'path': "/home/ron/Python/Camera/camera_processor.py",
                'venv_path': "/home/ron/Python/Camera/my_venv"
            },
            # Add more scripts here following the same format:
            # {
            #     'name': "My New Script",
            #     'path': "/path/to/your/new_script.py",
            #     'venv_path': "/path/to/your/new_project/my_env"
            # },
        ]
        # --- END SCRIPT CONFIGURATION ---

        self._create_widgets()

    def _create_widgets(self):
        main_frame = ttk.Frame(self.master, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Header Row
        ttk.Label(main_frame, text="Script Name", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(main_frame, text="Status", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Label(main_frame, text="Action", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Label(main_frame, text="Details (Venv & Script Path)", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=3, padx=5, pady=5, sticky="w")

        self.script_rows = {} # To store references to widgets for each script

        for i, script_info in enumerate(self.scripts_config):
            row_num = i + 1
            script_name = script_info['name']
            script_path = script_info['path']
            venv_path = script_info['venv_path']

            # Script Name Label
            ttk.Label(main_frame, text=script_name).grid(row=row_num, column=0, padx=5, pady=2, sticky="w")

            # Status Label
            status_label = ttk.Label(main_frame, text="Stopped", foreground="red")
            status_label.grid(row=row_num, column=1, padx=5, pady=2, sticky="w")

            # Action Button (Start/Stop)
            action_button = ttk.Button(main_frame, text="Start",
                                       command=lambda s=script_name: self._toggle_script(s))
            action_button.grid(row=row_num, column=2, padx=5, pady=2, sticky="w")

            # Details Label (Venv Path and Script Path)
            details_text = f"Venv: {venv_path}\nScript: {script_path}"
            details_label = ttk.Label(main_frame, text=details_text, wraplength=400, justify=tk.LEFT)
            details_label.grid(row=row_num, column=3, padx=5, pady=2, sticky="w")

            self.script_rows[script_name] = {
                'status_label': status_label,
                'action_button': action_button,
                'script_info': script_info # Store the original config
            }

        # Configure columns to expand
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=0) # Status doesn't need much width
        main_frame.grid_columnconfigure(2, weight=0) # Button doesn't need much width
        main_frame.grid_columnconfigure(3, weight=2) # Details can take more space

        # Add a "Close All" button at the bottom
        close_all_button = ttk.Button(main_frame, text="Close All Scripts", command=self._close_all_scripts)
        close_all_button.grid(row=len(self.scripts_config) + 1, column=0, columnspan=4, pady=20)

    def _get_terminal_command(self, script_venv_python, script_full_path, script_directory, venv_activate_path):
        """Constructs the appropriate terminal command based on OS,
           ensuring the correct directory and venv activation."""

        python_run_command = f'"{script_venv_python}" "{script_full_path}"'
        terminal_command_list = None

        if sys.platform.startswith('linux'):
            # Simplified: Remove exec bash and the prompt message.
            # The terminal will close automatically when python_run_command finishes.
            full_command_for_bash = (
                f'echo "Sourcing Venv: {venv_activate_path}..." && '
                f'source "{venv_activate_path}" && ' # Activates venv for the script
                f'echo "Executing Python script: {script_full_path}" && '
                f'{python_run_command}' # <--- No exec bash or final message here
            )
            # This list is passed directly to xfce4-terminal when shell=False.
            terminal_command_list = [
                'xfce4-terminal',
                '--working-directory', script_directory,
                '-x',              # Flag for xfce4-terminal to execute a command
                'bash',            # The command to execute (bash)
                '-c',              # Flag for bash to interpret the next argument as a command string
                full_command_for_bash # The actual command string for bash to run
            ]
            print(f"DEBUG: Linux Full Command String for Bash (inner): {full_command_for_bash}")
            print(f"DEBUG: xfce4-terminal command list (outer, for Popen): {terminal_command_list}")

        elif sys.platform == 'darwin': # macOS
            venv_bin_dir = os.path.dirname(venv_activate_path)
            escaped_script_directory = script_directory.replace('"', '\\"')
            escaped_venv_bin_dir = venv_bin_dir.replace('"', '\\"')
            escaped_venv_python = script_venv_python.replace('"', '\\"')
            escaped_script_full_path = script_full_path.replace('"', '\\"')

            # Same logic: remove exec bash and final prompt message for auto-close
            script_content = (
                f'cd \\"{escaped_script_directory}\\" && '
                f'export PATH=\\"{escaped_venv_bin_dir}:\\$PATH\\" && '
                f'echo "Running {escaped_script_full_path}..." && '
                f'\\"{escaped_venv_python}\\" \\"{escaped_script_full_path}\\"'
            )
            # For macOS, 'do script' usually implies keeping the window open.
            # To auto-close, we might need 'do script ... ; delay 1 ; tell application "Terminal" to close front window'
            # Or use 'do shell script' without opening a new terminal if the script doesn't need interactive output.
            # Let's keep this as is for now and focus on Linux, unless you're on Mac primarily.
            terminal_command_list = ['osascript', '-e', f'tell application "Terminal" to do script "{script_content}"']

        elif sys.platform == 'win32':
            venv_activate_bat = os.path.join(os.path.dirname(script_venv_python), 'activate.bat')
            # For Windows, '/k' keeps the cmd window open. '/c' would close it automatically.
            full_command_in_terminal = (
                f'cd /d "{script_directory}" && '
                f'call "{venv_activate_bat}" && '
                f'echo "Running {script_full_path}..." && '
                f'{python_run_command}'
            )
            # Change '/k' to '/c' for auto-close on Windows
            terminal_command_list = ['start', 'cmd.exe', '/c', full_command_in_terminal]

        else:
            messagebox.showerror("OS Not Supported", "This OS is not officially supported for launching terminals.")
            return None

        return terminal_command_list

    def _toggle_script(self, script_name):
        if script_name in self.running_scripts:
            self._stop_script(script_name)
        else:
            self._start_script(script_name)

    def _start_script(self, script_name):
        script_info = next((s for s in self.scripts_config if s['name'] == script_name), None)
        if not script_info:
            messagebox.showerror("Error", f"Configuration for '{script_name}' not found.")
            return

        script_path = script_info['path']
        venv_path = script_info['venv_path']
        script_directory = os.path.dirname(script_path)

        # Construct the path to the Python executable inside the venv
        if sys.platform == 'win32':
            venv_python = os.path.join(venv_path, 'Scripts', 'python.exe')
            venv_activate_path = os.path.join(venv_path, 'Scripts', 'activate.bat')
        else: # Linux/macOS
            venv_python = os.path.join(venv_path, 'bin', 'python3') # Or 'python'
            venv_activate_path = os.path.join(venv_path, 'bin', 'activate')


        if not os.path.exists(venv_python):
            messagebox.showerror("Error", f"Virtual environment Python executable not found:\n{venv_python}\n"
                                          f"Please check venv_path in script configuration for '{script_name}'.")
            return
        if not os.path.exists(script_path):
            messagebox.showerror("Error", f"Script file not found:\n{script_path}\n"
                                          f"Please check 'path' in script configuration for '{script_name}'.")
            return
        if not os.path.exists(venv_activate_path):
             messagebox.showwarning("Warning", f"Virtual environment activate script not found:\n{venv_activate_path}\n"
                                              f"The script will still run in the venv, but the terminal prompt might not show '(venv)'.")


        try:
            terminal_command = self._get_terminal_command(venv_python, script_path, script_directory, venv_activate_path)
            if terminal_command is None:
                return # OS not supported

            # Use shell=True if the command is a single string that needs shell parsing (e.g., 'cd && ...')
            # For Linux and Windows, our constructed command string requires shell=True.
            # For macOS, osascript command is passed as a list, which sometimes allows shell=False,
            # but the complex string we build inside osascript often still needs shell=True for the inner shell command.
            #process = subprocess.Popen(terminal_command, shell=True)
            process = subprocess.Popen(terminal_command, shell=False) # <--- THIS IS THE KEY CHANGE!
            self.running_scripts[script_name] = {'process': process, 'script_info': script_info}
            self._update_script_ui(script_name, "Running", "green", "Stop")
            print(f"Started '{script_name}' (PID: {process.pid})")
        except FileNotFoundError as e:
            messagebox.showerror("Launch Error", f"Terminal command or script not found: {e}\n"
                                                  "Ensure your terminal emulator (e.g., xfce4-terminal, cmd) is in your system's PATH.")
        except Exception as e:
            messagebox.showerror("Launch Error", f"Failed to start '{script_name}': {e}")

    def _stop_script(self, script_name):
        if script_name in self.running_scripts:
            process_info = self.running_scripts[script_name]
            process = process_info['process']
            try:
                # Terminate the process. This sends a SIGTERM on Unix, or closes the window on Windows.
                # If the script doesn't handle SIGTERM gracefully, you might need process.kill()
                process.terminate()
                process.wait(timeout=5) # Wait for process to terminate, with a timeout
                print(f"Stopped '{script_name}' (PID: {process.pid})")
            except subprocess.TimeoutExpired:
                print(f"Process '{script_name}' (PID: {process.pid}) did not terminate gracefully, killing it.")
                process.kill() # Force kill if terminate fails
                process.wait()
            except Exception as e:
                messagebox.showerror("Stop Error", f"Failed to stop '{script_name}': {e}")
            finally:
                del self.running_scripts[script_name]
                self._update_script_ui(script_name, "Stopped", "red", "Start")
        else:
            messagebox.showinfo("Info", f"'{script_name}' is not currently running.")

    def _update_script_ui(self, script_name, status_text, status_color, button_text):
        if script_name in self.script_rows:
            self.script_rows[script_name]['status_label'].config(text=status_text, foreground=status_color)
            self.script_rows[script_name]['action_button'].config(text=button_text)
            self.master.update_idletasks() # Force UI update

    def _close_all_scripts(self):
        if not self.running_scripts:
            messagebox.showinfo("Info", "No scripts are currently running.")
            return

        confirm = messagebox.askyesno("Close All", "Are you sure you want to close all running scripts?")
        if confirm:
            scripts_to_stop = list(self.running_scripts.keys()) # Iterate over a copy as dict changes during loop
            for script_name in scripts_to_stop:
                self._stop_script(script_name)
            messagebox.showinfo("Close All", "All scripts have been requested to stop.")

    def on_closing(self):
        if self.running_scripts:
            confirm = messagebox.askyesno("Exit OneRing",
                                          "Some scripts are still running. Do you want to close them and exit OneRing?")
            if confirm:
                self._close_all_scripts()
                self.master.destroy()
            else:
                pass # Don't destroy if user cancels
        else:
            self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ScriptManagerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
