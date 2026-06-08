import tkinter as tk
from tkinter import ttk


class JointControl:

    def __init__(self, parent, joint_id, rowNo, colNo):
        self.parent = parent
        self.joint_id = joint_id

        # This variable holds the actual offset value
        self.offset = tk.IntVar(value=0)
        self.loop_id = None  # Tracks the repeating loop

        # --- UI Elements ---
        # Label for the Joint
        self.lbl_name = ttk.Label(
            parent, text=f"Joint {joint_id} Offset:"
        )
        self.lbl_name.grid(row=rowNo, column=colNo+0, padx=10, pady=10, sticky="w")

        # Minus Button
        self.btn_minus = ttk.Button(parent, text="-", width=3)
        self.btn_minus.grid(row=rowNo, column=colNo+1, padx=5, pady=10)
        # Bind press and release events
        self.btn_minus.bind("<ButtonPress-1>", lambda e: self.start_loop(-1))
        self.btn_minus.bind("<ButtonRelease-1>", lambda e: self.stop_loop())

        # Value Display
        self.lbl_val = ttk.Label(
            parent,
            textvariable=self.offset,
            # font=("Arial", 12, "bold"),
            # width=6,
            # anchor="center",
        )
        self.lbl_val.grid(row=rowNo, column=colNo+2, padx=10, pady=10)

        # Plus Button
        self.btn_plus = ttk.Button(parent, text="+", width=3)
        self.btn_plus.grid(row=rowNo, column=colNo+3, padx=5, pady=10)
        # Bind press and release events
        self.btn_plus.bind("<ButtonPress-1>", lambda e: self.start_loop(1))
        self.btn_plus.bind("<ButtonRelease-1>", lambda e: self.stop_loop())

    def start_loop(self, direction):
        """Starts the continuous increment/decrement loop."""
        self.update_value(direction)

    def update_value(self, direction):
        """Changes the value and schedules the next change if button is still held."""
        self.offset.set(self.offset.get() + direction)

        # 100 milliseconds delay between steps. Adjust this to make it faster/slower!
        self.loop_id = self.parent.after(100, lambda: self.update_value(direction))

    def stop_loop(self):
        """Cancels the loop when the button is released."""
        if self.loop_id:
            self.parent.after_cancel(self.loop_id)
            self.loop_id = None