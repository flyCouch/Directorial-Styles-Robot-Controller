import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import sys

class SimpleScriptLauncher:
    def __init__(self, master):
        self.master = master
        master.title("oneRing")
        master.geometry("600x400") 
        
        # --- COSMETIC CHANGE: SET LIGHT GREEN BACKGROUND ---
        master.config(bg="lightgreen")
        style = ttk.Style()
        style.configure("Green.TFrame", background="lightgreen")
        # -------------------------------------------------
        
        # --- SCRIPT CONFIGURATION ---
        self.scripts_config = [
            {
                'name': "Joystick Configurator",
                'path': "/home/ron/ronPython/joysticks/joystickConfig21.py",
                'venv_path': "/home/ron/ronPython/joysticks/venv"
            },
            {
                'name': "Camera Calibation",
                'path': "/home/ron/ronPython/camCalibrate/camCalibration2.py",
                'venv_path': "/home/ron/ronPython/camCalibrate/venv"
            },
            {
                'name': "April Tag",
                'path': "/home/ron/ronPython/AprilTagTests/AprilTagTest14.py",
                'venv_path': "/home/ron/ronPython/AprilTagTests/venv" 
            },
            {
                'name': "robotDirector",
                'path': "/home/ron/ronPython/omniWheeler/scripts/robotDirector370.py",
                'venv_path': "/home/ron/ronPython/omniWheeler/venv"
            },
            {
                'name': "Image to Gcode",
                'path': "/home/ron/ronPython/image2Gcode/image2GcodeContouring15.py",
                'venv_path': "/home/ron/ronPython/image2Gcode/venv"
            },
        ]
        # --- END SCRIPT CONFIGURATION ---

        self._create_widgets()

    def _create_widgets(self):
        # Apply the custom style here
        main_frame = ttk.Frame(self.master, padding="15", style="Green.TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Set background for the top label to match the frame
        ttk.Label(main_frame, 
                  text="Select a Script to Launch (in a New Terminal)", 
                  font=("TkDefaultFont", 12, "bold"),
                  background="lightgreen" # Set background of label explicitly
                  ).pack(pady=10)
        
        for script_info in self.scripts_config:
            script_name = script_info['name']
            
            # Action Button (Start)
            action_button = ttk.Button(main_frame, text=f"Launch {script_name}",
                                       command=lambda s=script_name: self._start_script(s))
            action_button.pack(fill=tk.X, padx=20, pady=5)
            
            # Display Venv/Path details below the button
            details_text = f"Venv: {os.path.basename(script_info['venv_path'])} | Script: {os.path.basename(script_info['path'])}"
            ttk.Label(main_frame, 
                      text=details_text, 
                      foreground='gray',
                      background="lightgreen" # Set background of label explicitly
                      ).pack(fill=tk.X, padx=20)
            

    def _get_terminal_command(self, script_venv_python, script_full_path, script_directory, venv_activate_path):
        """Constructs the appropriate terminal command for the OS."""
        
        python_run_command = f'"{script_venv_python}" "{script_full_path}"'

        if sys.platform.startswith('linux'):
            # The inner command string for bash
            full_command_for_bash = (
                f'echo "Sourcing Venv: {os.path.basename(venv_activate_path)}..." && '
                f'source "{venv_activate_path}" && ' 
                f'echo "Executing: {os.path.basename(script_full_path)}" && '
                f'{python_run_command} || echo "--- SCRIPT FAILED OR FINISHED. PRESS ENTER TO CLOSE ---" && read' 
            )
            # This list is passed directly to xfce4-terminal
            terminal_command_list = [
                'xfce4-terminal',
                '--working-directory', script_directory,
                '-x',              # Flag for xfce4-terminal to execute a command
                'bash',            # The command to execute (bash)
                '-c',              # Flag for bash to interpret the next argument as a command string
                full_command_for_bash # The actual command string for bash to run
            ]
            
        else:
            messagebox.showerror("OS Not Supported", "This script is optimized for Linux (xfce4-terminal).")
            return None

        return terminal_command_list

    def _start_script(self, script_name):
        script_info = next((s for s in self.scripts_config if s['name'] == script_name), None)
        if not script_info:
            messagebox.showerror("Error", f"Configuration for '{script_name}' not found.")
            return

        script_path = script_info['path']
        venv_path = script_info['venv_path']
        script_directory = os.path.dirname(script_path)

        # Construct the path to the Python executable inside the venv
        venv_python = os.path.join(venv_path, 'bin', 'python3') 
        venv_activate_path = os.path.join(venv_path, 'bin', 'activate')


        if not os.path.exists(venv_python):
            messagebox.showerror("Error", f"Venv Python executable not found:\n{venv_python}")
            return
        if not os.path.exists(script_path):
            messagebox.showerror("Error", f"Script file not found:\n{script_path}")
            return


        try:
            terminal_command = self._get_terminal_command(venv_python, script_path, script_directory, venv_activate_path)
            if terminal_command is None:
                return # OS not supported

            subprocess.Popen(terminal_command, shell=False) 
            print(f"Launched '{script_name}' in new terminal.")
        except FileNotFoundError as e:
            messagebox.showerror("Launch Error", f"Terminal command or file not found: {e}\n"
                                                 "Ensure 'xfce4-terminal' is available in your system's PATH.")
        except Exception as e:
            messagebox.showerror("Launch Error", f"Failed to start '{script_name}': {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = SimpleScriptLauncher(root)
    root.mainloop()

# Copyright Ron Lyttle 2025
