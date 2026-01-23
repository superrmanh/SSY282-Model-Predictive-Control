from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

# Animation deps (only used if animate=True and a figure exists)
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.lines import Line2D


@dataclass
class CartPole:
    # --- public "plant" parameters ---
    animate: bool = True
    animationFig: Optional[plt.Figure] = None
    pauseTime: float = 0.01 #
    notebook_mode: bool = False

    g: float = 9.81         # [m/s^2] gravitational acceleration.
    l: float = 0.5          # [m] the distance from the pole-cart attachment to the pole's center of mass (half length).
    m: float = 0.1          # [kg] mass of the pole.
    m_cart: float = 1.0     # [kg] mass of the cart.
    mu_cart: float = 0.0    # [kg/s] friction force on the cart (default 0.01)
    mu_pole: float = 0.0    # [kg*m^2/s] friction force on the pole (default 0.001)
    k: float = 1.0 / 3.0    # constant to compute moment of inertia of the pole as I=k*m*l^2. For a pendulum (with mass only at the top), k=1, for a solid pole with uniform mass, k=1/3.

    # state x = [s, ds/dt, theta, dtheta/dt]
    x: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float)) # field --> avoid sharing x among different instances

    # input u = [F, Fd]
    u: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))

    # --- plotting-only ---
    cart_width: float = 0.4     # [m]
    cart_heigh: float = 0.2     # [m]
    wheel_radius: float = 0.05  # [m]
    pole_width: float = 0.02    # [m]
    color: tuple = (0.7, 0.7, 0.7)

    # Axis object
    _ax: Optional[plt.Axes] = field(default=None, init=False, repr=False)
    # Title text
    _state_text: Optional[plt.Text] = field(default=None, init=False, repr=False)
    # Cart-Pole geometric parts
    _cart_patch: Optional[Rectangle] = field(default=None, init=False, repr=False)
    _pole_patch: Optional[Polygon] = field(default=None, init=False, repr=False)
    _joint_patch: Optional[Circle] = field(default=None, init=False, repr=False)
    _wheel_patches: list = field(default_factory=list, init=False, repr=False)
    _wheel_spokes: list = field(default_factory=list, init=False, repr=False)  # list[list[Line2D]]

    # Methods
    #===============================
    def simulate(self, F: float = 0.0, dt: float = 0.1, *, disturbance: float = 0.0,title: Optional[str] = None) -> np.ndarray:
        """
        Simulate the plant for a time interval dt using force F and optional disturbance Fd.
        Returns x_next and also updates self.x.
        """
        # Check if inputs are valid
        if not np.isfinite(F) or not np.isfinite(dt) or dt == 0.0 or not np.isfinite(disturbance):
            raise ValueError("Invalid F, dt, or disturbance.")

        Fd = float(disturbance) # Disturbance force
        x_next = self.rk4(self.x, np.array([F, Fd], dtype=float), dt) # RK4 step
        self.x = x_next
    
        if self.animate and self._fig_is_valid():

            if self._ax is None:
                self.plot()
            if title is not None:
                self._ax.set_title(title)

            self.plot()

            if self.notebook_mode:
                self._notebook_display()
            else:
                plt.pause(self.pauseTime)



        return x_next

    # ------------------ animation / plotting ------------------
    def _notebook_display(self):
        """Safely display the figure in a Jupyter notebook."""
        try:
            from IPython.display import clear_output, display
        except ImportError:
            raise RuntimeError(
                "notebook_mode=True requires IPython/Jupyter to be installed."
            )
        clear_output(wait=True)
        display(self.animationFig)
    def set_animation_figure(self, fig: Optional[plt.Figure] = None, notebook_mode :bool = False) -> plt.Figure:
        """Assign (or create) a Matplotlib figure used for animation."""
        if fig is None:
            fig = plt.figure(figsize= (10,6))
        self.notebook_mode = notebook_mode
        self.animationFig = fig
        # force re-init next plot call
        self._ax = None
        self._cart_patch = None
        self._pole_patch = None
        self._joint_patch = None
        self._wheel_patches = []
        self._wheel_spokes = []
        return fig

    def plot(self) -> None:
        if self.animationFig is None:
            self.set_animation_figure()

        if self._ax is None or self._cart_patch is None:
            self._init_plot_objects()

        s = float(self.x[0])
        theta = float(self.x[2])

        # "local" geometry constants (cart frame)
        height_pole = self.cart_heigh + self.wheel_radius
        r = self.wheel_radius
        w = self.cart_width

        # base transforms: defined in cart-local coordinates, then shifted by s
        cart_T = mtransforms.Affine2D().translate(s, 0.0) # translate by s
        pole_T = mtransforms.Affine2D().rotate_around(0.0, height_pole, theta).translate(s, 0.0) # translate by s + rotate by theta

        # wheel rotation angle 
        phi = -s / r
        left_c = (-w / 2 + 2 * r, r)
        right_c = (w / 2 - 2 * r, r)
        left_T = mtransforms.Affine2D().rotate_around(left_c[0], left_c[1], phi).translate(s, 0.0)
        right_T = mtransforms.Affine2D().rotate_around(right_c[0], right_c[1], phi).translate(s, 0.0)

        # apply transforms (compose with ax.transData)
        axT = self._ax.transData
        self._cart_patch.set_transform(cart_T + axT)
        self._joint_patch.set_transform(cart_T + axT)
        self._pole_patch.set_transform(pole_T + axT)

        # wheels + spokes
        wheel_Ts = [left_T, right_T]
        for i in range(2):
            self._wheel_patches[i].set_transform(wheel_Ts[i] + axT)
            for ln in self._wheel_spokes[i]:
                ln.set_transform(wheel_Ts[i] + axT)

        # Update overlay text
        if self._state_text is not None:
            self._state_text.set_text(
                f"s = {s:+.3f} m\n"
                f"theta = {theta:+.3f} rad"
            )

        self.animationFig.canvas.draw_idle()

    def _init_plot_objects(self) -> None:
        # axes
        self.animationFig.clf() # clear the drawings but keep the plotting object
        self._ax = self.animationFig.add_subplot(1, 1, 1)
        self._ax.set_title("Cart-pole system")

        # local geometry
        w = self.cart_width
        h = self.cart_heigh
        r = self.wheel_radius
        lp = self.l
        pw = self.pole_width
        height_pole = h + r

        # cart body (local coords)
        self._cart_patch = Rectangle(
            xy=(-w / 2, r), # coordinate of the lower left corner of the box
            width=w,
            height=h,
            facecolor=self.color,
            edgecolor="k",
            linewidth=1.0,
            joinstyle="round",
        )
        self._ax.add_patch(self._cart_patch)

        # wheels (local coords)
        left_center = (-w / 2 + 2 * r, r)
        right_center = (w / 2 - 2 * r, r)
        self._wheel_patches = [
            Circle(left_center, radius=r, facecolor=self.color, edgecolor="k", linewidth=1.0),
            Circle(right_center, radius=r, facecolor=self.color, edgecolor="k", linewidth=1.0),
        ]
        for c in self._wheel_patches:
            self._ax.add_patch(c)

        # spokes (as lines in local coords; rotated via transform)
        n_spokes = 6
        self._wheel_spokes = [[], []] # lists for the left wheel and right wheel spokes
        for idx, (cx, cy) in enumerate([left_center, right_center]):
            for j in range(1, n_spokes + 1):
                a = j * 2 * np.pi / n_spokes
                # each line has start (cx,yx) and end (cx+ r cos(a), cy + r sin(a))
                xdata = [cx, cx + r * np.cos(a)]
                ydata = [cy, cy + r * np.sin(a)]
                ln = Line2D(xdata, ydata, color="k", linewidth=1.0)
                self._ax.add_line(ln)
                self._wheel_spokes[idx].append(ln)

        # pole (local coords), to specify a rectangle (polygon) object we need 4 points, one for each corner
        pole_xy = np.array( 
            [
                [-pw, height_pole + 0.0], # bottom left
                [pw, height_pole + 0.0],  # bottom right
                [pw, height_pole + 2 * lp], # top right
                [-pw, height_pole + 2 * lp], # top left
            ],
            dtype=float,
        )
        self._pole_patch = Polygon(pole_xy, closed=True, facecolor=self.color, edgecolor="k", linewidth=1.0)
        self._ax.add_patch(self._pole_patch)

        # joint (pole axis)
        self._joint_patch = Circle((0.0, height_pole), radius=1.5 * pw, facecolor="k", edgecolor="k")
        self._ax.add_patch(self._joint_patch)

        # plotting 
        self._ax.set_aspect("equal", adjustable="box")
        # hide axis ticks and borders, (keep bottom)
        self._ax.spines["top"].set_visible(False)
        self._ax.spines["right"].set_visible(False)
        self._ax.spines["left"].set_visible(False)
        self._ax.get_yaxis().set_visible(False)
        self._ax.spines["bottom"].set_position(("data", 0.0))
        self._ax.set_xlim(-4, 4)
        self._ax.set_ylim(h - 2 * self.l + r - 0.08, h + 2 * self.l + r + 0.08)
        # State overlay (axes coordinates: fixed to top-left of the axes)
        self._state_text = self._ax.text(
            0.02, 0.98, "",                 # x,y in axes fraction coords
            transform=self._ax.transAxes,   # makes it stay in the corner
            va="top", ha="left",
            fontsize=12,
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none")
        )

    def _fig_is_valid(self) -> bool:
        if self.animationFig is None:
            return False
        try:
            return plt.fignum_exists(self.animationFig.number)
        except Exception:
            return False

    # ------------------ dynamics ------------------
    # Fourth order Runge-Kutta solver
    def rk4(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        subintervals = 4
        h = dt / subintervals
        x_next = np.array(x, dtype=float, copy=True)
        for _ in range(subintervals):
            k1 = self.xdot(x_next, u)
            k2 = self.xdot(x_next + 0.5 * h * k1, u)
            k3 = self.xdot(x_next + 0.5 * h * k2, u)
            k4 = self.xdot(x_next + h * k3, u)
            x_next = x_next + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        return x_next
    # evaluates dx/dt = f(x,u)
    def xdot(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # state and control variables
        Ds, theta, Dtheta = x[1], x[2], x[3]
        F, Fd = u
        # some parameters
        M = self.m_cart + self.m
        c = np.cos(theta)
        s = np.sin(theta)
        denom = (1 + self.k) * M - self.m * c**2

        # evaluate dx/dt = f(x,u)
        f = np.zeros(4, dtype=float)
        f[0] = Ds
        f[1] = ((self.k + 1) * (F + Fd - self.mu_cart * Ds - self.l * self.m * s * Dtheta**2)
            + c * (-Fd * c + self.g * self.m * s + self.mu_pole * Dtheta / self.l)) / denom
        f[2] = Dtheta
        f[3] = (self.g * M * s + c * (F - self.m_cart * Fd / self.m - self.l * self.m * s * Dtheta**2 - self.mu_cart * Ds)
            - self.mu_pole * M * Dtheta / (self.m * self.l)) / (self.l * denom)

        return f
'''
Original Matlab code created by Nikolce Murgovski 2025
Python translation by Mohamed Abrash
'''