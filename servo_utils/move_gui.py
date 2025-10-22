import tkinter as tk

class JointControl:
    def __init__(self, master, name, min_val, max_val, row):
        self.name = name
        self.min_val = min_val
        self.max_val = max_val
        self.current_angle = tk.DoubleVar() # Variable to hold the angle

        # 1. DOF Name Label
        tk.Label(master, text=name).grid(row=row, column=0, padx=5, pady=2)

        # 2. Min Value Label
        tk.Label(master, text=str(min_val)).grid(row=row, column=1, padx=5, pady=2)

        # 3. Slider (Scale)
        self.slider = tk.Scale(
            master,
            from_=min_val,
            to=max_val,
            orient=tk.HORIZONTAL,
            resolution=0.1, # Set resolution for fine control
            variable=self.current_angle,
            command=self.update_entry # Link slider movement to update input box
        )
        self.slider.grid(row=row, column=2, padx=5, pady=2, sticky="ew")

        # 4. Max Value Label
        tk.Label(master, text=str(max_val)).grid(row=row, column=3, padx=5, pady=2)

        # 5. Input Box (Entry)
        self.entry = tk.Entry(master, width=6, textvariable=self.current_angle)
        self.entry.grid(row=row, column=4, padx=5, pady=2)
        # Link entry box change to update the slider
        self.entry.bind('<Return>', self.update_slider)

    def update_entry(self, *args):
        # Called when slider moves: ensures input box matches slider
        # (Tkinter's textvariable often handles this automatically, but a manual sync is safer)
        pass # The tk.DoubleVar should sync them automatically

    def update_slider(self, *args):
        # Called when user hits Enter in the input box: ensures slider matches box
        try:
            val = float(self.entry.get())
            if self.min_val <= val <= self.max_val:
                self.slider.set(val)
        except ValueError:
            # Handle invalid input
            pass

def create_gui():
    root = tk.Tk()
    root.title("12 DOF Joint Controller")

    # Example DOF data (Name, Min, Max)
    dof_data = [
        ("L1 Hip X", -45, 45), ("L1 Hip Y", -30, 30), ("L1 Knee", -90, 0),
        ("L2 Hip X", -45, 45), ("L2 Hip Y", -30, 30), ("L2 Knee", -90, 0),
        # ... 6 more entries ...
        ("L4 Hip X", -45, 45), ("L4 Hip Y", -30, 30), ("L4 Knee", -90, 0)
    ]
    
    # List to store all joint controllers
    joint_controllers = []

    # Create the 12 joint rows
    for i, (name, min_val, max_val) in enumerate(dof_data):
        controller = JointControl(root, name, min_val, max_val, row=i)
        joint_controllers.append(controller)

    # Global Reset Button
    def global_reset():
        # Set all sliders/entries to a neutral position (e.g., 0)
        for controller in joint_controllers:
            controller.current_angle.set(0) # Assumes 0 is within range

    tk.Button(root, text="GLOBAL RESET", command=global_reset).grid(
        row=len(dof_data), column=0, columnspan=5, pady=10
    )

    root.mainloop()

create_gui()