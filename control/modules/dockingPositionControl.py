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
        # self.min_arc_length = 0.001
        # Fixed limits for the arc-length controls
        self.min_s_l = 0.001
        self.max_s_l = 20.0

        self.min_s_r = 0.001
        self.max_s_r = 20.0

        self.min_circumference_1 = self.min_s_l + self.min_s_r
        self.max_circumference_1 = self.max_s_l + self.max_s_r

        self.max_circumference_2 = 40.0

        self.min_radius_1 = self.min_circumference_1 / (2.0 * np.pi)
        self.max_radius_1 = self.max_circumference_1 / (2.0 * np.pi)
        self.max_radius_2 = self.max_circumference_2 / (2.0 * np.pi)

        # Avoid y = 0 when directly controlling P because
        # r = (x^2 + y^2) / (2y) is undefined there.
        self.min_point_y = 0.001

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

        initial_p_x = initial_r_1 * np.cos(initial_theta)
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

        # Prevent callbacks from recursively triggering each other.
        self.updating_controls = False

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
            minimum=self.min_s_l + self.min_s_r,
            maximum=self.max_s_l + self.max_s_r,
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

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------
    def calculate_point_from_arc_lengths(self):
        """Calculate P from the current s_l and s_r values."""
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
        self.p_y.set(max(p_y, self.min_point_y))


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

        # Ensure that the requested circumference can be represented by
        # valid values of both s_l and s_r.
        new_circumference = np.clip(
            new_circumference,
            self.min_circumference_1,
            self.max_circumference_1
        )

        # Desired value based on preserving the current ratio.
        desired_s_l = ratio_l * new_circumference

        # Because s_r = C - s_l, s_l must satisfy:
        #
        #   min_s_l <= s_l <= max_s_l
        #   min_s_r <= C - s_l <= max_s_r
        #
        # Combining these gives the following feasible interval.
        minimum_allowed_s_l = max(
            self.min_s_l,
            new_circumference - self.max_s_r
        )

        maximum_allowed_s_l = min(
            self.max_s_l,
            new_circumference - self.min_s_r
        )

        new_s_l = np.clip(
            desired_s_l,
            minimum_allowed_s_l,
            maximum_allowed_s_l
        )

        new_s_r = new_circumference - new_s_l

        self.s_l.set(new_s_l)
        self.s_r.set(new_s_r)
        self.c_1.set(new_circumference)

        self.synchronise_from_arc_lengths()


    def synchronise_from_point(self, requested_x, requested_y):
        """
        Calculate C, r, s_l and s_r from a directly selected point P.

        The first circle must:
            1. Be centred at (0, r)
            2. Pass through the origin
            3. Pass through P

        This gives:
            r = (x^2 + y^2) / (2y)
        """
        max_radius = self.max_radius_1

        # Ensure y remains in the valid range.
        y = np.clip(
            requested_y,
            self.min_point_y,
            2.0 * max_radius
        )

        # For the chosen y, constrain x so that the required radius
        # does not exceed max_radius.
        #
        # x^2 + y^2 <= 2 * max_radius * y
        maximum_x_squared = max(
            0.0,
            2.0 * max_radius * y - y**2
        )
        maximum_abs_x = np.sqrt(maximum_x_squared)

        x = np.clip(
            requested_x,
            -maximum_abs_x,
            maximum_abs_x
        )

        radius = (x**2 + y**2) / (2.0 * y)
        radius = np.clip(radius, self.min_radius_1, self.max_radius_1)

        circumference = 2.0 * np.pi * radius

        # Angle relative to the circle centre.
        theta = np.arctan2(
            y - radius,
            x
        )

        # Convert the angle into the fraction represented by s_r.
        #
        # theta = 2*pi*(s_r/C) - pi/2
        angular_fraction = (
            (theta + np.pi / 2.0) % (2.0 * np.pi)
        ) / (2.0 * np.pi)

        s_r = circumference * angular_fraction
        s_l = circumference - s_r

        # Constrain s_l while keeping s_l + s_r equal to the calculated
        # circumference.
        minimum_allowed_s_l = max(
            self.min_s_l,
            circumference - self.max_s_r
        )

        maximum_allowed_s_l = min(
            self.max_s_l,
            circumference - self.min_s_r
        )

        if minimum_allowed_s_l <= maximum_allowed_s_l:
            s_l = np.clip(
                s_l,
                minimum_allowed_s_l,
                maximum_allowed_s_l
            )

            s_r = circumference - s_l

        else:
            # The circumference calculated from the requested point cannot
            # be represented using the configured arc-length limits.
            circumference = np.clip(
                circumference,
                self.min_circumference_1,
                self.max_circumference_1
            )

            s_l = np.clip(
                s_l,
                self.min_s_l,
                self.max_s_l
            )

            s_r = np.clip(
                circumference - s_l,
                self.min_s_r,
                self.max_s_r
            )

            # Recalculate the circumference and point from the constrained
            # arc lengths to keep all variables consistent.
            circumference = s_l + s_r
            radius = circumference / (2.0 * np.pi)

            theta = (
                2.0 * np.pi * s_r / circumference
                - np.pi / 2.0
            )

            x = radius * np.cos(theta)
            y = radius + radius * np.sin(theta)

        self.p_x.set(x)
        self.p_y.set(y)
        self.c_1.set(circumference)
        self.s_l.set(s_l)
        self.s_r.set(s_r)

        # A direct point change changes the circumference. If the lock
        # is enabled, update its stored value to the new circumference.
        if self.lock_circumference.get():
            self.locked_circumference = circumference


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
            adjusted_s_l = np.clip(
                self.s_l.get(),
                self.min_s_l,
                self.max_s_l
            )

            if (
                self.lock_circumference.get()
                and self.locked_circumference is not None
            ):
                circumference = self.locked_circumference

                # Constrain s_l so that the resulting value of
                # s_r = C - s_l also lies within its limits.
                minimum_allowed_s_l = max(
                    self.min_s_l,
                    circumference - self.max_s_r
                )

                maximum_allowed_s_l = min(
                    self.max_s_l,
                    circumference - self.min_s_r
                )

                if minimum_allowed_s_l <= maximum_allowed_s_l:
                    adjusted_s_l = np.clip(
                        adjusted_s_l,
                        minimum_allowed_s_l,
                        maximum_allowed_s_l
                    )

                    adjusted_s_r = circumference - adjusted_s_l

                    self.s_l.set(adjusted_s_l)
                    self.s_r.set(adjusted_s_r)
                else:
                    # This should only occur if the stored circumference
                    # is incompatible with the configured limits.
                    adjusted_s_l = np.clip(
                        adjusted_s_l,
                        self.min_s_l,
                        self.max_s_l
                    )

                    adjusted_s_r = np.clip(
                        circumference - adjusted_s_l,
                        self.min_s_r,
                        self.max_s_r
                    )

                    self.s_l.set(adjusted_s_l)
                    self.s_r.set(adjusted_s_r)

                    self.locked_circumference = (
                        adjusted_s_l + adjusted_s_r
                    )

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
            adjusted_s_r = np.clip(
                self.s_r.get(),
                self.min_s_r,
                self.max_s_r
            )

            if (
                self.lock_circumference.get()
                and self.locked_circumference is not None
            ):
                circumference = self.locked_circumference

                # Constrain s_r so that the resulting value of
                # s_l = C - s_r also lies within its limits.
                minimum_allowed_s_r = max(
                    self.min_s_r,
                    circumference - self.max_s_l
                )

                maximum_allowed_s_r = min(
                    self.max_s_r,
                    circumference - self.min_s_l
                )

                if minimum_allowed_s_r <= maximum_allowed_s_r:
                    adjusted_s_r = np.clip(
                        adjusted_s_r,
                        minimum_allowed_s_r,
                        maximum_allowed_s_r
                    )

                    adjusted_s_l = circumference - adjusted_s_r

                    self.s_r.set(adjusted_s_r)
                    self.s_l.set(adjusted_s_l)
                else:
                    adjusted_s_r = np.clip(
                        adjusted_s_r,
                        self.min_s_r,
                        self.max_s_r
                    )

                    adjusted_s_l = np.clip(
                        circumference - adjusted_s_r,
                        self.min_s_l,
                        self.max_s_l
                    )

                    self.s_r.set(adjusted_s_r)
                    self.s_l.set(adjusted_s_l)

                    self.locked_circumference = (
                        adjusted_s_l + adjusted_s_r
                    )

            else:
                self.s_r.set(adjusted_s_r)

            self.synchronise_from_arc_lengths()

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_c_1_changed(self, _value=None):
        """
        Change circle 1's circumference while preserving s_l:s_r.
        """
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            new_circumference = np.clip(
                self.c_1.get(),
                self.min_circumference_1,
                self.max_circumference_1
            )

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
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            self.synchronise_from_point(
                requested_x=self.p_x.get(),
                requested_y=self.p_y.get()
            )
        finally:
            self.updating_controls = False

        self.update_plot()

    def on_p_y_changed(self, _value=None):
        if self.updating_controls:
            return

        self.updating_controls = True

        try:
            requested_x = self.p_x.get()
            requested_y = np.clip(
                self.p_y.get(),
                self.min_point_y,
                2.0 * self.max_radius_1
            )

            # For the requested x, determine the valid y interval for
            # circles whose radius is no greater than max_radius_1.
            x = np.clip(
                requested_x,
                -self.max_radius_1,
                self.max_radius_1
            )

            root_term = np.sqrt(
                max(0.0, self.max_radius_1**2 - x**2)
            )

            minimum_valid_y = max(
                self.min_point_y,
                self.max_radius_1 - root_term
            )
            maximum_valid_y = (
                self.max_radius_1 + root_term
            )

            requested_y = np.clip(
                requested_y,
                minimum_valid_y,
                maximum_valid_y
            )

            self.synchronise_from_point(
                requested_x=x,
                requested_y=requested_y
            )

        finally:
            self.updating_controls = False

        self.update_plot()

    def on_c_2_changed(self, _value=None):
        if not self.updating_controls:
            self.update_plot()

    def on_lock_changed(self):
        if self.lock_circumference.get():
            self.locked_circumference = (
                self.s_l.get() + self.s_r.get()
            )
        else:
            self.locked_circumference = None

        self.update_plot()

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------
    def reset_values(self):
        self.updating_controls = True

        try:
            self.lock_circumference.set(False)
            self.locked_circumference = None

            self.s_l.set(5.0)
            self.s_r.set(5.0)
            self.c_2.set(8.0)

            self.synchronise_from_arc_lengths()

        finally:
            self.updating_controls = False

        self.update_plot()

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------
    def update_plot(self):
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

        phi = np.linspace(0.0, 2.0 * np.pi, 600)

        # The origin is at angle -pi/2 around circle 1.
        origin_angle = -np.pi / 2.0

        # Left arc:
        # Travel anticlockwise from P to the origin.
        #
        # theta is in the interval [-pi/2, 3*pi/2), so the equivalent
        # origin angle reached anticlockwise is 3*pi/2.
        left_arc_angles = np.linspace(
            theta,
            origin_angle + 2.0 * np.pi,
            400
        )

        left_arc_x = radius * np.cos(left_arc_angles)
        left_arc_y = radius + radius * np.sin(left_arc_angles)

        # Right arc:
        # Travel clockwise from P back to the origin.
        right_arc_angles = np.linspace(
            theta,
            origin_angle,
            400
        )

        right_arc_x = radius * np.cos(right_arc_angles)
        right_arc_y = radius + radius * np.sin(right_arc_angles)

        circle_2_x = radius_2 * np.cos(phi)
        circle_2_y = radius_2 + radius_2 * np.sin(phi)

        self.ax.clear()

        # Anticlockwise arc from P to the origin.
        # Its length is s_l.
        self.ax.plot(
            left_arc_x,
            left_arc_y,
            color="tab:blue",
            linewidth=3.5,
            label=f"Left arc: s_l = {s_l:.3f}"
        )

        # Clockwise arc from P to the origin.
        # Its length is s_r.
        self.ax.plot(
            right_arc_x,
            right_arc_y,
            color="tab:green",
            linewidth=3.5,
            label=f"Right arc: s_r = {s_r:.3f}"
        )

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

        # Circle centres
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

        # Point P
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

        # self.ax.plot(
        #     [0.0, p_x],
        #     [radius, p_y],
        #     color="crimson",
        #     linewidth=1.5,
        #     alpha=0.75
        # )

        self.ax.annotate(
            "P",
            xy=(p_x, p_y),
            xytext=(8, 8),
            textcoords="offset points",
            color="crimson",
            fontsize=11,
            fontweight="bold"
        )

        # Origin
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

        # Fixed plot limits based on maximum permitted circle sizes.
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

        # Numerical consistency check
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