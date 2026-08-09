import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class CircleVisualiser:
    def __init__(self, root):
        self.root = root
        self.root.title("Interactive Circle Visualiser")
        self.root.geometry("1200x780")

        # --------------------------------------------------------------
        # Control limits
        # --------------------------------------------------------------
        self.min_s_l = 0.001
        self.max_s_l = 20.0

        self.min_s_r = 0.001
        self.max_s_r = 20.0

        # Circle 1 circumference limits are derived from the arc limits.
        self.min_circumference_1 = (
            self.min_s_l + self.min_s_r
        )
        self.max_circumference_1 = (
            self.max_s_l + self.max_s_r
        )

        self.max_circumference_2 = 40.0

        # Radius limits derived from the circumference limits.
        self.min_radius_1 = (
            self.min_circumference_1 / (2.0 * np.pi)
        )
        self.max_radius_1 = (
            self.max_circumference_1 / (2.0 * np.pi)
        )
        self.max_radius_2 = (
            self.max_circumference_2 / (2.0 * np.pi)
        )

        # y must remain positive because:
        #
        #     r = (x^2 + y^2) / (2y)
        #
        # is undefined when y = 0.
        self.min_point_y = 0.001

        # Floating-point comparison tolerance.
        self.tolerance = 1.0e-9

        # --------------------------------------------------------------
        # Initial values
        # --------------------------------------------------------------
        initial_s_l = 5.0
        initial_s_r = 5.0

        initial_c_1 = initial_s_l + initial_s_r
        initial_r_1 = initial_c_1 / (2.0 * np.pi)

        initial_theta = (
            2.0 * np.pi * initial_s_r / initial_c_1
            - np.pi / 2.0
        )

        initial_p_x = (
            initial_r_1 * np.cos(initial_theta)
        )
        initial_p_y = (
            initial_r_1
            + initial_r_1 * np.sin(initial_theta)
        )

        # --------------------------------------------------------------
        # Tkinter variables
        # --------------------------------------------------------------
        self.s_l = tk.DoubleVar(value=initial_s_l)
        self.s_r = tk.DoubleVar(value=initial_s_r)

        self.c_1 = tk.DoubleVar(value=initial_c_1)
        self.c_2 = tk.DoubleVar(value=8.0)

        self.p_x = tk.DoubleVar(value=initial_p_x)
        self.p_y = tk.DoubleVar(value=initial_p_y)

        self.lock_circumference = tk.BooleanVar(value=False)
        self.locked_circumference = None

        # Prevent callbacks from recursively triggering one another.
        self.updating_controls = False

        # Store the last fully valid state. If a point-slider movement
        # violates a limit, all circle-1 controls are restored to this
        # state.
        self.last_valid_state = {
            "s_l": initial_s_l,
            "s_r": initial_s_r,
            "c_1": initial_c_1,
            "p_x": initial_p_x,
            "p_y": initial_p_y
        }

        # --------------------------------------------------------------
        # Main layout
        # --------------------------------------------------------------
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main_frame = ttk.Frame(root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=0)
        main_frame.rowconfigure(0, weight=1)

        # --------------------------------------------------------------
        # Matplotlib figure
        # --------------------------------------------------------------
        self.figure = Figure(figsize=(8.0, 7.2), dpi=100)
        self.ax = self.figure.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(
            self.figure,
            master=main_frame
        )
        self.canvas.get_tk_widget().grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 15)
        )

        # --------------------------------------------------------------
        # Controls
        # --------------------------------------------------------------
        controls = ttk.LabelFrame(
            main_frame,
            text="Circle controls",
            padding=15
        )
        controls.grid(row=0, column=1, sticky="ns")
        controls.columnconfigure(0, weight=1)

        self.create_slider(
            controls,
            label="Left arc length, s_l",
            variable=self.s_l,
            minimum=self.min_s_l,
            maximum=self.max_s_l,
            row=0,
            callback=self.on_s_l_changed
        )

        self.create_slider(
            controls,
            label="Right arc length, s_r",
            variable=self.s_r,
            minimum=self.min_s_r,
            maximum=self.max_s_r,
            row=1,
            callback=self.on_s_r_changed
        )

        self.create_slider(
            controls,
            label="Circle 1 circumference, C",
            variable=self.c_1,
            minimum=self.min_circumference_1,
            maximum=self.max_circumference_1,
            row=2,
            callback=self.on_c_1_changed
        )

        self.lock_checkbox = ttk.Checkbutton(
            controls,
            text="Keep circle 1 circumference constant",
            variable=self.lock_circumference,
            command=self.on_lock_changed
        )
        self.lock_checkbox.grid(
            row=3,
            column=0,
            sticky="w",
            pady=(10, 5)
        )

        ttk.Separator(
            controls,
            orient="horizontal"
        ).grid(
            row=4,
            column=0,
            sticky="ew",
            pady=12
        )

        ttk.Label(
            controls,
            text="Direct point P controls",
            font=("TkDefaultFont", 10, "bold")
        ).grid(
            row=5,
            column=0,
            sticky="w",
            pady=(0, 4)
        )

        self.create_slider(
            controls,
            label="Point x-coordinate, P_x",
            variable=self.p_x,
            minimum=-self.max_radius_1,
            maximum=self.max_radius_1,
            row=6,
            callback=self.on_p_x_changed
        )

        self.create_slider(
            controls,
            label="Point y-coordinate, P_y",
            variable=self.p_y,
            minimum=self.min_point_y,
            maximum=2.0 * self.max_radius_1,
            row=7,
            callback=self.on_p_y_changed
        )

        ttk.Separator(
            controls,
            orient="horizontal"
        ).grid(
            row=8,
            column=0,
            sticky="ew",
            pady=12
        )

        self.create_slider(
            controls,
            label="Circle 2 circumference, C_2",
            variable=self.c_2,
            minimum=0.1,
            maximum=self.max_circumference_2,
            row=9,
            callback=self.on_c_2_changed
        )

        ttk.Separator(
            controls,
            orient="horizontal"
        ).grid(
            row=10,
            column=0,
            sticky="ew",
            pady=12
        )

        self.first_circle_info = ttk.Label(
            controls,
            justify="left"
        )
        self.first_circle_info.grid(
            row=11,
            column=0,
            sticky="w",
            pady=4
        )

        self.point_info = ttk.Label(
            controls,
            justify="left"
        )
        self.point_info.grid(
            row=12,
            column=0,
            sticky="w",
            pady=4
        )

        self.second_circle_info = ttk.Label(
            controls,
            justify="left"
        )
        self.second_circle_info.grid(
            row=13,
            column=0,
            sticky="w",
            pady=4
        )

        ttk.Button(
            controls,
            text="Reset values",
            command=self.reset_values
        ).grid(
            row=14,
            column=0,
            sticky="ew",
            pady=(15, 0)
        )

        self.update_plot()

    # ------------------------------------------------------------------
    # GUI helper
    # ------------------------------------------------------------------
    def create_slider(
        self,
        parent,
        label,
        variable,
        minimum,
        maximum,
        row,
        callback
    ):
        """Create a labelled horizontal slider."""
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=6)
        frame.columnconfigure(0, weight=1)

        ttk.Label(
            frame,
            text=label
        ).grid(
            row=0,
            column=0,
            sticky="w"
        )

        value_label = ttk.Label(
            frame,
            width=9,
            anchor="e"
        )
        value_label.grid(
            row=0,
            column=1,
            sticky="e",
            padx=(10, 0)
        )

        slider = ttk.Scale(
            frame,
            from_=minimum,
            to=maximum,
            orient="horizontal",
            variable=variable,
            command=callback,
            length=270
        )
        slider.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(3, 0)
        )

        variable.trace_add(
            "write",
            lambda *_args, var=variable, label=value_label:
            label.configure(text=f"{var.get():.4f}")
        )

        value_label.configure(text=f"{variable.get():.4f}")

        return slider

    # ------------------------------------------------------------------
    # State-management helpers
    # ------------------------------------------------------------------
    def remember_current_state(self):
        """
        Store the current circle-1 controls as the last valid state.
        """
        self.last_valid_state = {
            "s_l": self.s_l.get(),
            "s_r": self.s_r.get(),
            "c_1": self.c_1.get(),
            "p_x": self.p_x.get(),
            "p_y": self.p_y.get()
        }

    def restore_last_valid_state(self):
        """
        Restore all circle-1 controls to the last valid state.

        This is called while self.updating_controls is True, so the
        programmatic changes do not trigger additional calculations.
        """
        self.s_l.set(self.last_valid_state["s_l"])
        self.s_r.set(self.last_valid_state["s_r"])
        self.c_1.set(self.last_valid_state["c_1"])
        self.p_x.set(self.last_valid_state["p_x"])
        self.p_y.set(self.last_valid_state["p_y"])

        if self.lock_circumference.get():
            self.locked_circumference = (
                self.last_valid_state["c_1"]
            )

    def apply_point_state(self, state):
        """
        Apply a valid point-derived state to all circle-1 controls.
        """
        self.s_l.set(state["s_l"])
        self.s_r.set(state["s_r"])
        self.c_1.set(state["circumference"])
        self.p_x.set(state["p_x"])
        self.p_y.set(state["p_y"])

        if self.lock_circumference.get():
            self.locked_circumference = state["circumference"]

        self.remember_current_state()

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------
    def calculate_point_from_arc_lengths(self):
        """
        Calculate circle 1 and point P from the current s_l and s_r.
        """
        s_l = self.s_l.get()
        s_r = self.s_r.get()

        circumference = s_l + s_r
        radius = circumference / (2.0 * np.pi)

        theta = (
            2.0 * np.pi * s_r / circumference
            - np.pi / 2.0
        )

        p_x = radius * np.cos(theta)
        p_y = radius + radius * np.sin(theta)

        return circumference, radius, theta, p_x, p_y

    def synchronise_from_arc_lengths(self):
        """
        Update C and P after s_l or s_r has changed.
        """
        circumference, _, _, p_x, p_y = (
            self.calculate_point_from_arc_lengths()
        )

        self.c_1.set(circumference)
        self.p_x.set(p_x)

        # Do not clamp p_y here. Clamping p_y independently would make
        # the point controls inconsistent with s_l and s_r.
        self.p_y.set(p_y)

        self.remember_current_state()

    def synchronise_from_circumference(self, new_circumference):
        """
        Change circle 1's circumference while preserving the current
        s_l:s_r ratio as closely as possible.

        The fixed minimum and maximum values of both arc lengths are
        respected.
        """
        current_s_l = self.s_l.get()
        current_s_r = self.s_r.get()
        current_total = current_s_l + current_s_r

        if current_total <= 0.0:
            ratio_l = 0.5
        else:
            ratio_l = current_s_l / current_total

        ratio_l = np.clip(ratio_l, 0.0, 1.0)

        new_circumference = float(np.clip(
            new_circumference,
            self.min_circumference_1,
            self.max_circumference_1
        ))

        # Desired value based on retaining the current ratio.
        desired_s_l = ratio_l * new_circumference

        # s_l must remain inside its own limits and leave a valid
        # value for s_r = C - s_l.
        minimum_allowed_s_l = max(
            self.min_s_l,
            new_circumference - self.max_s_r
        )

        maximum_allowed_s_l = min(
            self.max_s_l,
            new_circumference - self.min_s_r
        )

        new_s_l = float(np.clip(
            desired_s_l,
            minimum_allowed_s_l,
            maximum_allowed_s_l
        ))

        new_s_r = new_circumference - new_s_l

        self.s_l.set(new_s_l)
        self.s_r.set(new_s_r)
        self.c_1.set(new_circumference)

        self.synchronise_from_arc_lengths()

    def synchronise_from_point(self, requested_x, requested_y):
        """
        Attempt to calculate C, r, s_l, and s_r from a requested P.

        The movement is accepted only if all resulting values remain
        within their configured limits. If any limit is violated, the
        entire movement is rejected and all controls are returned to
        their last valid values.

        Returns
        -------
        bool
            True if the requested point was accepted.
            False if the requested movement was rejected.
        """
        tolerance = self.tolerance

        x = float(requested_x)
        y = float(requested_y)

        # --------------------------------------------------------------
        # Validate the requested coordinates without clipping
        # --------------------------------------------------------------
        if not np.isfinite(x) or not np.isfinite(y):
            self.restore_last_valid_state()
            return False

        if y < self.min_point_y - tolerance:
            self.restore_last_valid_state()
            return False

        # --------------------------------------------------------------
        # Calculate the circle implied by the requested point
        # --------------------------------------------------------------
        radius = (x**2 + y**2) / (2.0 * y)

        if not np.isfinite(radius):
            self.restore_last_valid_state()
            return False

        if (
            radius < self.min_radius_1 - tolerance
            or radius > self.max_radius_1 + tolerance
        ):
            self.restore_last_valid_state()
            return False

        circumference = 2.0 * np.pi * radius

        if (
            circumference
            < self.min_circumference_1 - tolerance
            or circumference
            > self.max_circumference_1 + tolerance
        ):
            self.restore_last_valid_state()
            return False

        # --------------------------------------------------------------
        # Calculate the arc lengths implied by the requested point
        # --------------------------------------------------------------
        theta = np.arctan2(
            y - radius,
            x
        )

        # From:
        #
        # theta = 2*pi*(s_r/C) - pi/2
        #
        # obtain the fraction of the circumference represented by s_r.
        angular_fraction = (
            (theta + np.pi / 2.0) % (2.0 * np.pi)
        ) / (2.0 * np.pi)

        s_r = circumference * angular_fraction
        s_l = circumference - s_r

        # Remove tiny floating-point errors exactly at a limit.
        if abs(s_l - self.min_s_l) <= tolerance:
            s_l = self.min_s_l

        if abs(s_l - self.max_s_l) <= tolerance:
            s_l = self.max_s_l

        if abs(s_r - self.min_s_r) <= tolerance:
            s_r = self.min_s_r

        if abs(s_r - self.max_s_r) <= tolerance:
            s_r = self.max_s_r

        # --------------------------------------------------------------
        # Reject the complete movement if either arc is invalid
        # --------------------------------------------------------------
        if (
            s_l < self.min_s_l - tolerance
            or s_l > self.max_s_l + tolerance
            or s_r < self.min_s_r - tolerance
            or s_r > self.max_s_r + tolerance
        ):
            self.restore_last_valid_state()
            return False

        # Apply only tiny numerical corrections after validation.
        s_l = float(np.clip(
            s_l,
            self.min_s_l,
            self.max_s_l
        ))

        s_r = float(np.clip(
            s_r,
            self.min_s_r,
            self.max_s_r
        ))

        # Recalculate every related quantity from the accepted arc
        # lengths. This keeps the plot and all sliders consistent.
        circumference = s_l + s_r
        radius = circumference / (2.0 * np.pi)

        theta = (
            2.0 * np.pi * s_r / circumference
            - np.pi / 2.0
        )

        accepted_x = radius * np.cos(theta)
        accepted_y = radius + radius * np.sin(theta)

        proposed_state = {
            "s_l": s_l,
            "s_r": s_r,
            "circumference": circumference,
            "p_x": accepted_x,
            "p_y": accepted_y
        }

        self.apply_point_state(proposed_state)

        return True

    # ------------------------------------------------------------------
    # Slider callbacks
    # ------------------------------------------------------------------
    def on_s_l_changed(self, _value=None):
        """
        Respond to movement of the s_l slider.

        If circumference locking is enabled, s_r is adjusted so that
        s_l + s_r remains constant.
        """
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            adjusted_s_l = float(np.clip(
                self.s_l.get(),
                self.min_s_l,
                self.max_s_l
            ))

            if (
                self.lock_circumference.get()
                and self.locked_circumference is not None
            ):
                circumference = self.locked_circumference

                # s_l must remain inside its limits and leave a valid
                # value for s_r = C - s_l.
                minimum_allowed_s_l = max(
                    self.min_s_l,
                    circumference - self.max_s_r
                )

                maximum_allowed_s_l = min(
                    self.max_s_l,
                    circumference - self.min_s_r
                )

                if minimum_allowed_s_l <= maximum_allowed_s_l:
                    adjusted_s_l = float(np.clip(
                        adjusted_s_l,
                        minimum_allowed_s_l,
                        maximum_allowed_s_l
                    ))

                    adjusted_s_r = (
                        circumference - adjusted_s_l
                    )

                    self.s_l.set(adjusted_s_l)
                    self.s_r.set(adjusted_s_r)

                else:
                    # Restore the last valid state if the locked
                    # circumference cannot satisfy the arc limits.
                    self.restore_last_valid_state()
                    return

            else:
                self.s_l.set(adjusted_s_l)

            self.synchronise_from_arc_lengths()

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_s_r_changed(self, _value=None):
        """
        Respond to movement of the s_r slider.

        If circumference locking is enabled, s_l is adjusted so that
        s_l + s_r remains constant.
        """
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            adjusted_s_r = float(np.clip(
                self.s_r.get(),
                self.min_s_r,
                self.max_s_r
            ))

            if (
                self.lock_circumference.get()
                and self.locked_circumference is not None
            ):
                circumference = self.locked_circumference

                # s_r must remain inside its limits and leave a valid
                # value for s_l = C - s_r.
                minimum_allowed_s_r = max(
                    self.min_s_r,
                    circumference - self.max_s_l
                )

                maximum_allowed_s_r = min(
                    self.max_s_r,
                    circumference - self.min_s_l
                )

                if minimum_allowed_s_r <= maximum_allowed_s_r:
                    adjusted_s_r = float(np.clip(
                        adjusted_s_r,
                        minimum_allowed_s_r,
                        maximum_allowed_s_r
                    ))

                    adjusted_s_l = (
                        circumference - adjusted_s_r
                    )

                    self.s_r.set(adjusted_s_r)
                    self.s_l.set(adjusted_s_l)

                else:
                    # Restore the last valid state if the locked
                    # circumference cannot satisfy the arc limits.
                    self.restore_last_valid_state()
                    return

            else:
                self.s_r.set(adjusted_s_r)

            self.synchronise_from_arc_lengths()

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_c_1_changed(self, _value=None):
        """
        Change circle 1's circumference while preserving the current
        s_l:s_r ratio as closely as possible.
        """
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            new_circumference = float(np.clip(
                self.c_1.get(),
                self.min_circumference_1,
                self.max_circumference_1
            ))

            self.synchronise_from_circumference(
                new_circumference
            )

            if self.lock_circumference.get():
                self.locked_circumference = (
                    self.s_l.get() + self.s_r.get()
                )

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_p_x_changed(self, _value=None):
        """
        Attempt to change P_x while retaining the last valid P_y.

        If the movement causes any arc or circle limit to be exceeded,
        the P_x slider and all related controls return to the previous
        valid state.
        """
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            requested_x = self.p_x.get()
            current_y = self.last_valid_state["p_y"]

            self.synchronise_from_point(
                requested_x=requested_x,
                requested_y=current_y
            )

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_p_y_changed(self, _value=None):
        """
        Attempt to change P_y while retaining the last valid P_x.

        If the movement causes any arc or circle limit to be exceeded,
        the P_y slider and all related controls return to the previous
        valid state.
        """
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            current_x = self.last_valid_state["p_x"]
            requested_y = self.p_y.get()

            self.synchronise_from_point(
                requested_x=current_x,
                requested_y=requested_y
            )

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_c_2_changed(self, _value=None):
        """Redraw circle 2 after its circumference changes."""
        if not self.updating_controls:
            self.update_plot()

    def on_lock_changed(self):
        """Enable or disable circle-1 circumference locking."""
        if self.lock_circumference.get():
            self.locked_circumference = (
                self.s_l.get() + self.s_r.get()
            )
        else:
            self.locked_circumference = None

        self.remember_current_state()
        self.update_plot()

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------
    def reset_values(self):
        """Restore the initial values and disable circumference lock."""
        self.updating_controls = True

        try:
            self.lock_circumference.set(False)
            self.locked_circumference = None

            default_s_l = float(np.clip(
                5.0,
                self.min_s_l,
                self.max_s_l
            ))

            default_s_r = float(np.clip(
                5.0,
                self.min_s_r,
                self.max_s_r
            ))

            self.s_l.set(default_s_l)
            self.s_r.set(default_s_r)
            self.c_2.set(8.0)

            self.synchronise_from_arc_lengths()

        finally:
            self.updating_controls = False

        self.update_plot()

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------
    def update_plot(self):
        """Redraw both circles, the coloured arcs, and point P."""
        s_l = self.s_l.get()
        s_r = self.s_r.get()

        circumference = s_l + s_r
        c_2 = self.c_2.get()

        radius = circumference / (2.0 * np.pi)
        radius_2 = c_2 / (2.0 * np.pi)

        theta = (
            2.0 * np.pi * s_r / circumference
            - np.pi / 2.0
        )

        p_x = radius * np.cos(theta)
        p_y = radius + radius * np.sin(theta)

        phi = np.linspace(
            0.0,
            2.0 * np.pi,
            600
        )

        # The origin is at -pi/2 around circle 1.
        origin_angle = -np.pi / 2.0

        # Anticlockwise arc from P to the origin. Its length is s_l.
        left_arc_angles = np.linspace(
            theta,
            origin_angle + 2.0 * np.pi,
            400
        )

        left_arc_x = radius * np.cos(left_arc_angles)
        left_arc_y = (
            radius + radius * np.sin(left_arc_angles)
        )

        # Clockwise arc from P to the origin. Its length is s_r.
        right_arc_angles = np.linspace(
            theta,
            origin_angle,
            400
        )

        right_arc_x = radius * np.cos(right_arc_angles)
        right_arc_y = (
            radius + radius * np.sin(right_arc_angles)
        )

        # Complete second circle.
        circle_2_x = radius_2 * np.cos(phi)
        circle_2_y = radius_2 + radius_2 * np.sin(phi)

        self.ax.clear()

        # --------------------------------------------------------------
        # Circle 1 coloured arcs
        # --------------------------------------------------------------
        self.ax.plot(
            left_arc_x,
            left_arc_y,
            color="tab:blue",
            linewidth=3.5,
            label=f"Left arc: s_l = {s_l:.3f}"
        )

        self.ax.plot(
            right_arc_x,
            right_arc_y,
            color="tab:green",
            linewidth=3.5,
            label=f"Right arc: s_r = {s_r:.3f}"
        )

        # --------------------------------------------------------------
        # Circle 2
        # --------------------------------------------------------------
        self.ax.plot(
            circle_2_x,
            circle_2_y,
            color="tab:orange",
            linewidth=2.5,
            linestyle="--",
            label=(
                f"Circle 2: C₂ = {c_2:.3f}, "
                f"r₂ = {radius_2:.3f}"
            )
        )

        # --------------------------------------------------------------
        # Circle centres
        # --------------------------------------------------------------
        self.ax.scatter(
            [0.0],
            [radius],
            color="tab:blue",
            marker="x",
            s=80,
            linewidths=2,
            zorder=5
        )

        self.ax.scatter(
            [0.0],
            [radius_2],
            color="tab:orange",
            marker="x",
            s=80,
            linewidths=2,
            zorder=5
        )

        # --------------------------------------------------------------
        # Point P
        # --------------------------------------------------------------
        self.ax.scatter(
            [p_x],
            [p_y],
            color="crimson",
            s=90,
            edgecolor="white",
            linewidth=1.0,
            zorder=6,
            label=f"P = ({p_x:.3f}, {p_y:.3f})"
        )

        self.ax.annotate(
            "P",
            xy=(p_x, p_y),
            xytext=(8, 8),
            textcoords="offset points",
            color="crimson",
            fontsize=11,
            fontweight="bold"
        )

        # --------------------------------------------------------------
        # Origin and axes
        # --------------------------------------------------------------
        self.ax.scatter(
            [0.0],
            [0.0],
            color="black",
            s=30,
            zorder=5
        )

        self.ax.axhline(
            0.0,
            color="0.75",
            linewidth=0.8
        )

        self.ax.axvline(
            0.0,
            color="0.75",
            linewidth=0.8
        )

        # --------------------------------------------------------------
        # Fixed plot limits
        # --------------------------------------------------------------
        maximum_radius = max(
            self.max_radius_1,
            self.max_radius_2
        )

        margin = 0.5

        self.ax.set_xlim(
            -maximum_radius - margin,
            maximum_radius + margin
        )

        self.ax.set_ylim(
            -margin,
            2.0 * maximum_radius + margin
        )

        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_title(
            "Circles defined by circumference and point P"
        )
        self.ax.grid(True, alpha=0.25)
        self.ax.legend(loc="upper right")

        self.figure.tight_layout()
        self.canvas.draw_idle()

        # --------------------------------------------------------------
        # Information labels
        # --------------------------------------------------------------
        point_distance_from_centre = np.hypot(
            p_x,
            p_y - radius
        )

        self.first_circle_info.configure(
            text=(
                "First circle\n"
                f"C = {circumference:.4f}\n"
                f"r = {radius:.4f}\n"
                f"s_l / s_r = {s_l / s_r:.4f}"
            )
        )

        self.point_info.configure(
            text=(
                "Point P\n"
                f"P = ({p_x:.4f}, {p_y:.4f})\n"
                f"Angle = {theta:.4f} rad\n"
                f"Distance to centre = "
                f"{point_distance_from_centre:.4f}"
            )
        )

        self.second_circle_info.configure(
            text=(
                "Second circle\n"
                f"C_2 = {c_2:.4f}\n"
                f"r_2 = {radius_2:.4f}"
            )
        )


if __name__ == "__main__":
    root = tk.Tk()
    app = CircleVisualiser(root)
    root.mainloop()