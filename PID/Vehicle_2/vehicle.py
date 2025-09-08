import numpy as np
import matplotlib.pyplot as plt

class Vehicle():
    def __init__(
            self,
            l_f: float = 1.5,
            l_r: float = 1.0,
            max_steer_abs: float = 0.523,
            max_accel_abs: float = 2.000,
            ref_path: np.arrary = np.array([[-30,0,0],[30,0,0]]),
            delta_t: float = 0.05,
            visualize: bool = True,    
        ) -> None:
        # vehicle parameters
        self.l_r = l_r  # [m]
        self.l_f = l_f  # [m]
        self.wheel_base = l_f + l_r
        self.max_steer_abs = max_steer_abs
        self.max_accel_abs = max_accel_abs
        self.delta_t = delta_t
        self.ref_path = ref_path

        # visualize
        self.vehicle_w = 3.00
        self.vehicle_l = 4.00
        self.view_x_lim_min, self.view_x_lim_max = -20.0, 20.0
        self.view_y_lim_min, self.view_y_lim_max = -20.0, 20.0
        
        # reset enviroment
        self.visualize_flag = visualize
        self.reset()

    def reset(
            self,
            init_state: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0])
        ) -> None:

        # reset state variable
        self.state = init_state

        # Clear animation frames
        self.frames = []

        if self.visualize_flag:
            # prepare figure
            ## Main view
            self.fig = plt.figure(figsize=(9.9))
            self.main_ax = plt.subplot2grid((3,4), (0,0), rowspan=3, colspan=3)
            self.minimap_ax = plt.subplot2grid((3,4), (0,3))
            self.steer_ax = plt.subplot2grid((3,4), (1,3))
            self.accel_ax = plt.subplot2grid((3,4), (2,3))
            ## minimap
            self.minimap_ax.set_aspect('equal')
            self.minimap_ax.axis("off")
            ## steering angle view
            self.steer_ax.set_title("Steering Angle", fontsize="12")
            self.steer_ax.axis("off")
            ## acceleration view
            self.accel_ax.set_title("Acceleration", fontsize="12")
            self.accel_ax.axis("off")

            # apply tight layout
            self.fig.tight_layout()

    def update(
            self,
            u: np.ndarray,    
            delta_t: float = 0.0,
            append_frame: bool = True,
            vehicle_traj: np.array = np.empty(0),
        ) -> None:
        # keep previous states
        x, y, yaw, v = self.state

        # prepare params
        l = self.wheel_base
        lr = self.l_r
        dt = self.delta_t if delta_t == 0.0 else delta_t

        # limit control inputs
        steer = np.clip(u[0], -self.max_steer_abs, self.max_steer_abs)
        accel = np.clip(u[1], -self.max_accel_abs, self.max_accel_abs)

        # update state variables
        """" < CORE OF VEHICEL DYNAMICS > """
        beta = np.arctan(l_r/1 * np.tan(steer))
        """" < CORE OF VEHICEL DYNAMICS > """



