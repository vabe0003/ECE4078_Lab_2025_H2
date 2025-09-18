
import numpy as np

class Robot:
    def __init__(self, wheels_width, wheels_scale, camera_matrix, camera_dist):
        # State is a vector of [x,y,theta]'
        self.state = np.zeros((3,1))

        # Wheel parameters
        self.wheels_width = float(wheels_width)  # [m]
        self.wheels_scale = float(wheels_scale)  # [m/s per (tick/s)]

        # Camera parameters
        self.camera_matrix = camera_matrix
        self.camera_dist   = camera_dist

    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def drive(self, drive_meas):
        # left_speed & right_speed in ticks/s, dt in seconds
        v, w = self.convert_wheel_speeds(drive_meas.left_speed, drive_meas.right_speed)
        dt   = float(drive_meas.dt)

        th = float(self.state[2])
        eps = 1e-9

        if abs(w) < eps:
            # straight-line limit
            self.state[0] += np.cos(th) * v * dt
            self.state[1] += np.sin(th) * v * dt
            self.state[2] += w * dt
        else:
            th2 = th + w * dt
            self.state[0] += (v / w) * (np.sin(th2) - np.sin(th))
            self.state[1] += -(v / w) * (np.cos(th2) - np.cos(th))
            self.state[2] += w * dt

        # keep heading bounded even if EKF is paused
        self.state[2,0] = self._wrap_angle(self.state[2,0])

    def measure(self, markers, idx_list):
        # markers: 2 x N (world); returns stacked 2 x M body-frame measurements
        th = float(self.state[2])
        Rot_theta = np.block([[np.cos(th), -np.sin(th)],
                              [np.sin(th),  np.cos(th)]])
        robot_xy = self.state[0:2,:]

        measurements = []
        for idx in idx_list:
            marker = markers[:, idx:idx+1]         # world
            marker_bff = Rot_theta.T @ (marker - robot_xy)
            measurements.append(marker_bff)

        return np.concatenate(measurements, axis=1) if measurements else np.zeros((2,0))

    def convert_wheel_speeds(self, left_speed, right_speed):
        # Convert ticks/s -> m/s
        left_speed_m  = float(left_speed)  * self.wheels_scale
        right_speed_m = float(right_speed) * self.wheels_scale
        # v, w
        v = (left_speed_m + right_speed_m) / 2.0
        w = (right_speed_m - left_speed_m) / self.wheels_width
        return v, w

    # ---------------- Derivatives and Covariance ----------------

    def derivative_drive(self, drive_meas):
        # ∂f/∂x (discrete Jacobian of motion wrt robot state)
        DFx = np.eye(3)

        v, w = self.convert_wheel_speeds(drive_meas.left_speed, drive_meas.right_speed)
        dt   = float(drive_meas.dt)
        th   = float(self.state[2])
        th2  = th + w*dt

        eps = 1e-9
        if abs(w) < eps:
            # straight-line limit
            DFx[0, 2] = -v * dt * np.sin(th)
            DFx[1, 2] =  v * dt * np.cos(th)
        else:
            DFx[0, 2] = (v / w) * (np.cos(th2) - np.cos(th))
            DFx[1, 2] = (v / w) * (np.sin(th2) - np.sin(th))

        return DFx

    def derivative_measure(self, markers, idx_list):
        # H = ∂h/∂[x,y,θ, l1x,l1y, ...]
        n = 2*len(idx_list)
        m = 3 + 2*markers.shape[1]
        DH = np.zeros((n, m))

        robot_xy = self.state[0:2,:]
        th = float(self.state[2])
        Rot_theta  = np.block([[np.cos(th), -np.sin(th)],
                               [np.sin(th),  np.cos(th)]])
        DRot_theta = np.block([[-np.sin(th), -np.cos(th)],
                               [ np.cos(th), -np.sin(th)]])

        for i in range(len(idx_list)):
            j = idx_list[i]                     # landmark index in state
            lmj_world = markers[:, j:j+1]

            # robot x,y
            DH[2*i:2*i+2, 0:2] = -Rot_theta.T
            # robot θ
            DH[2*i:2*i+2, 2:3] = DRot_theta.T @ (lmj_world - robot_xy)
            # landmark (x,y)
            DH[2*i:2*i+2, 3+2*j:3+2*j+2] = Rot_theta.T

        return DH

    def covariance_drive(self, drive_meas):
        # Map wheel noise -> state noise
        # Jac1 = ∂[v, w]/∂[L, R]
        Jac1 = np.array([[ self.wheels_scale/2.0,  self.wheels_scale/2.0],
                         [-self.wheels_scale/self.wheels_width, self.wheels_scale/self.wheels_width]], dtype=float)

        v, w = self.convert_wheel_speeds(drive_meas.left_speed, drive_meas.right_speed)
        dt   = float(drive_meas.dt)
        th   = float(self.state[2])
        th2  = th + w*dt

        # Jac2 = ∂[x, y, θ]/∂[v, w]
        Jac2 = np.zeros((3,2), dtype=float)
        eps  = 1e-9
        if abs(w) < eps:
            # small-ω limits
            Jac2[0,0] = dt * np.cos(th)                    # ∂x/∂v
            Jac2[1,0] = dt * np.sin(th)                    # ∂y/∂v
            Jac2[0,1] = 0.5 * v * (dt**2) * (-np.sin(th))  # ∂x/∂w
            Jac2[1,1] = 0.5 * v * (dt**2) * ( np.cos(th))  # ∂y/∂w
        else:
            s1 = np.sin(th2) - np.sin(th)
            c1 = np.cos(th2) - np.cos(th)
            Jac2[0,0] =  s1 / w                             # ∂x/∂v
            Jac2[1,0] = -c1 / w                             # ∂y/∂v
            Jac2[0,1] = v * ((dt * np.cos(th2)) / w - s1 / (w**2))
            Jac2[1,1] = v * ((dt * np.sin(th2)) / w + c1 / (w**2))

        Jac2[2,0] = 0.0
        Jac2[2,1] = dt   # ∂θ/∂w  (always dt)

        # Chain rule: ∂[x,y,θ]/∂[L,R] = Jac2 @ Jac1
        Jac = Jac2 @ Jac1

        # Wheel covariance (variances in ticks^2/s^2)
        cov_wheels = np.diag((float(drive_meas.left_cov), float(drive_meas.right_cov)))
        cov_state  = Jac @ cov_wheels @ Jac.T
        return cov_state
