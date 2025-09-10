import numpy as np

class LQRController():
    def __init__(
            self,
            wheel_base: float = 2.5, # [m] wheel_base
            Q: np.ndarray = np.diag([1.0, 1.0]), # weight matrix for state variables
            R: np.ndarray = np.diag([1.0]), # weight matrix for control inputs
            ref_path: np.ndarray = np.array([[0.0, 0.0, 0.0, 1.0], [10.0, 0.0, 0.0, 1.0]]),
    ) -> None:
        """initialize lqr controller for path-tracking"""
        # vehicle parameters
        self.l = wheel_base # [m] wheel base

        # weight matrices for LQR
        self.Q = Q # weight matrix for state variables
        self.R = R # weight matrix for control inputs

        # ref_path info
        self.ref_path = ref_path
        self.prev_waypoints_idx = 0

    def calc_control_input(self, observed_x: np.ndarray, delta_t: float) -> float:
        """calculate control input"""

        # set vehicle state variables from observation
        x = observed_x[0]
        y = observed_x[1]
        yaw = observed_x[2]
        v = observed_x[3]

        # get the waypoint closest to current vehicle position
        _, ref_x, ref_y, ref_yaw, _ = self._get_nearest_waypoint(x, y, update_prev_idx=True)
        if self.prev_waypoints_idx >= self.ref_path.shape[0]-1:
            print("[ERROR] Reached the end of the reference path.")
            raise IndexError

        # which side of the reference path is the car on, the right or the left?
        ## algorithm : http://www.hptown.com/ucad/Ufb00009.htm
        x1, y1 = ref_x, ref_y
        x2, y2 = ref_x + 1.0 * np.cos(ref_yaw), ref_y + 1.0 * np.sin(ref_yaw)
        vx, vy = x2 - x1, y2 - y1
        wx, wy =  x - x1,  y - y1
        s = vx * wy - vy * wx # s>0 : vehicle is on the left of the path, s<0 : vehicle is on the left of the path,

        # get tracking error
        y_e = np.sign(s) * np.sqrt((ref_x-x)**2 + (ref_y-y)**2) # lateral error 
        theta_e = yaw - ref_yaw # heading error
        theta_e = np.arctan2(np.sin(theta_e), np.cos(theta_e)) # normalize heading error to [-pi, pi]

        # define A, B matrices and solve algebraic riccati equation to get feedback gain matrix f for LQR
        delta_ref = 0.0 # [rad] reference steering angle (assuming the reference path is straight line)
        A = np.array([
            [0, v],
            [0, 0],
        ])
        B = np.array([
            [0],
            [v / (self.l * (np.cos(delta_ref))**2)],
        ])

        # calculate control input
        P = self.solve_are(A, B, self.Q, self.R)
        f = np.linalg.inv(self.R) @ B.T @ P
        steer_cmd = -f @ np.array([y_e, theta_e])

        return steer_cmd[0].real # TODO : why does steer_cmd have imaginary part?

    def solve_are(self, A, B, Q, R):
        """solve algebraic riccati equation with the Arimoto-Potter algorithm
        Ref: https://qiita.com/trgkpc/items/8210927d5b035912a153
        """
        # define hamiltonian matrix
        H = np.block([[A, -B @ np.linalg.inv(R) @ B.T],
                    [-Q , -A.T]])

        # solve eigenvalue problem
        eigenvalue, w = np.linalg.eig(H)

        # define Y and Z, which are used to calculate P
        Y_, Z_ = [], []
        n = len(w[0])//2

        # sort eigenvalues
        index_array = sorted([i for i in range(2*n)],
            key = lambda x:eigenvalue[x].real)

        # choose n eigenvalues which have smaller real part
        for i in index_array[:n]:
            Y_.append(w.T[i][:n])
            Z_.append(w.T[i][n:])
        Y = np.array(Y_).T
        Z = np.array(Z_).T

        # calculate P
        if np.linalg.det(Y) != 0:
            return Z @ np.linalg.inv(Y)
        else:
            print("Warning: Y is not regular matrix. Result may be wrong!") # TODO : need to consider mathmatical meaning of this case.
            return Z @ np.linalg.pinv(Y)

    def _get_nearest_waypoint(self, x: float, y: float, update_prev_idx: bool = False):
        """search the closest waypoint to the vehicle on the reference path"""
        SEARCH_IDX_LEN = 200 # [points] forward search range
        prev_idx = self.prev_waypoints_idx
        dx = [x - ref_x for ref_x in self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 0]]
        dy = [y - ref_y for ref_y in self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 1]]
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        min_d = min(d)
        nearest_idx = d.index(min_d) + prev_idx

        # get reference values of the nearest waypoint
        ref_x = self.ref_path[nearest_idx,0]
        ref_y = self.ref_path[nearest_idx,1]
        ref_yaw = self.ref_path[nearest_idx,2]
        ref_v = self.ref_path[nearest_idx,3]

        # update nearest waypoint index if necessary
        if update_prev_idx:
            self.prev_waypoints_idx = nearest_idx 

        return nearest_idx, ref_x, ref_y, ref_yaw, ref_v



