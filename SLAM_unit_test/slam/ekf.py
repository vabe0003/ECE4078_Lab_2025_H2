import numpy as np
from mapping_utils import MappingUtils
import cv2
import math
import pygame

class EKF:
    """
    EKF for landmark‑based SLAM.
    State order: [x; y; theta; l1x; l1y; ...; lnx; lny]

    ── QUICK TUNING GUIDE (tell your friend) ─────────────────────────────────────
    * BASE_MEAS_STD_M (R base):
        - Start: 0.03 (3 cm)
        - Increase to 0.035–0.04 if good measurements get rejected too often
          (few updates) or detector is noisy.
        - Decrease to 0.02–0.025 if map is jitter‑free and you want tighter fit.

    * GATE_CHI2 (Mahalanobis gating threshold, 2D):
        - Start: 5.991  (95% conf)
        - Raise to 7.378 (97.5%) if too many good measurements are rejected.
        - Lower to 4.0–5.0 if obvious outliers slip through.

    * RANGE_INFLATE (how much R grows with distance):
        - Start: 0.8
        - Increase (→1.0–1.2) if far markers look noisy / cause jumps.
        - Decrease (→0.5) if far markers are still pretty stable.

    * RESIDUAL_CAP (max multiple to inflate R by residual size):
        - Start: 3.0
        - Lower (→2.0) if map jitters (too reactive); raise (→3.5) if still stiff.

    * Q_STAB (extra process noise stabilizer for pose):
        - Start: 0.02 (was 0.01)
        - Increase (→0.03–0.05) if odometry is over‑confident (slow to correct).
        - Decrease (→0.01) if the map becomes too loose/noisy.

    * INIT_LM_STD_M (landmark init std if not using smart init):
        - Start: 0.40 m (so P block ≈ 0.16 on diag).
        - Increase if new landmarks jump around before settling.
        - Decrease if new landmarks converge too slowly.

    * USE_SMART_LM_INIT:
        - True: initialize new landmark covariance from the current measurement
          covariance rotated into world (usually faster & cleaner).
        - False: use a fixed INIT_LM_STD_M.

    * OPTIONAL_ALIGNMENT:
        - If your evaluation GT frame needs a rotation/translation, you can enable
          this to bake the SE(2) alignment into the map frame. Leave disabled unless
          you specifically want to align to an external GT.
    ─────────────────────────────────────────────────────────────────────────────
    """

    # ===================== TUNABLES (edit here) =====================
    BASE_MEAS_STD_M = 0.03    # (R base) per‑axis std in meters (3 cm is a good start)
    GATE_CHI2       = 5.991   # 95% chi^2 in 2D. Raise to 7.378 (97.5%) if dropping too much.
    RANGE_INFLATE   = 0.8     # how fast R grows with distance (0.5–1.2 typical)
    RESIDUAL_CAP    = 3.0     # maximum extra inflation based on residual size (2.0–3.5)
    Q_STAB          = 0.01    # extra stabilizer on pose process noise (0.01–0.05)
    INIT_LM_STD_M   = 0.40    # fixed landmark init std (used if not smart init)
    USE_SMART_LM_INIT = True  # True -> use rotated measurement covariance for new LM
    OPTIONAL_ALIGNMENT = False  # set True only if you want to align to an external GT
    ALIGN_ROT_RAD   = -1.7354400353556125  # used only if OPTIONAL_ALIGNMENT=True
    ALIGN_TX        = 0.2689604391944955   # used only if OPTIONAL_ALIGNMENT=True
    ALIGN_TY        = 0.11240062133992615  # used only if OPTIONAL_ALIGNMENT=True
    # ================================================================

    # 95% chi-square threshold for 2D Mahalanobis gating (kept for reference)
    CHI2_95_2D = 5.991

    @staticmethod
    def _wrap_angle(a):
        """Wrap angle to [-pi, pi)."""
        return (a + np.pi) % (2*np.pi) - np.pi

    def _adaptive_R(self, z_i, d2=None):
        """
        Per‑measurement covariance (2x2):
        - Base = BASE_MEAS_STD_M^2
        - Inflate with range (||z||) and residual size (Mahalanobis d^2)
        """
        Rb = np.diag([self.BASE_MEAS_STD_M**2, self.BASE_MEAS_STD_M**2])
        r = float(np.linalg.norm(z_i))
        scale_r   = 1.0 + self.RANGE_INFLATE * (min(r, 2.0)/1.0)**2  # cap at 2 m
        scale_res = 1.0 if d2 is None else min(self.RESIDUAL_CAP, 0.5 + d2/self.GATE_CHI2)
        return (scale_r * scale_res) * Rb

    def _gate_one(self, z_i, zhat_i, H_i, R_i):
        """Mahalanobis gate a single 2D measurement."""
        S_i = H_i @ self.P @ H_i.T + R_i
        v_i = z_i - zhat_i
        d2  = float(v_i.T @ np.linalg.solve(S_i, v_i))
        return (d2 <= self.GATE_CHI2), d2

    def __init__(self, robot):
        # State components
        self.robot = robot
        self.markers = np.zeros((2,0))
        self.taglist = []

        # Covariance matrix (robot only at start)
        self.P = np.zeros((3,3))
        self.init_lm_cov = 1e3  # legacy; not used if USE_SMART_LM_INIT=True
        self.robot_init_state = None
        self.lm_pics = []
        for i in range(1, 11):
            f_ = f'./pics/8bit/lm_{i}.png'
            self.lm_pics.append(pygame.image.load(f_))
        f_ = f'./pics/8bit/lm_unknown.png'
        self.lm_pics.append(pygame.image.load(f_))
        self.pibot_pic = pygame.image.load(f'./pics/8bit/pibot_top.png')

        # Optional alignment SE(2) to run EKF already in GT frame
        if self.OPTIONAL_ALIGNMENT:
            c, s = np.cos(self.ALIGN_ROT_RAD), np.sin(self.ALIGN_ROT_RAD)
            self.eval_R = np.array([[c, -s],[s, c]])
            self.eval_t = np.array([[self.ALIGN_TX],[self.ALIGN_TY]])
        else:
            self.eval_R = None
            self.eval_t = None
        
    def reset(self):
        self.robot.state = np.zeros((3, 1))
        self.markers = np.zeros((2,0))
        self.taglist = []
        self.P = np.zeros((3,3))
        self.robot_init_state = None

    def number_landmarks(self):
        return int(self.markers.shape[1])

    def get_state_vector(self):
        state = np.concatenate(
            (self.robot.state, np.reshape(self.markers, (-1,1), order='F')), axis=0)
        return state
    
    def set_state_vector(self, state):
        self.robot.state = state[0:3,:]
        self.markers = np.reshape(state[3:,:], (2,-1), order='F')
    
    def save_map(self, fname="slam_map.txt"):
        if self.number_landmarks() > 0:
            utils = MappingUtils(self.markers, self.P[3:,3:], self.taglist)
            utils.save(fname)

    def recover_from_pause(self, measurements):
        if not measurements:
            return False
        else:
            lm_new = np.zeros((2,0))
            lm_prev = np.zeros((2,0))
            tag = []
            for lm in measurements:
                if lm.tag in self.taglist:
                    lm_new = np.concatenate((lm_new, lm.position), axis=1)
                    tag.append(int(lm.tag))
                    lm_idx = self.taglist.index(lm.tag)
                    lm_prev = np.concatenate((lm_prev,self.markers[:,lm_idx].reshape(2, 1)), axis=1)
            if int(lm_new.shape[1]) > 2:
                R,t = self.umeyama(lm_new, lm_prev)
                theta = math.atan2(R[1][0], R[0][0])
                self.robot.state[:2]=t[:2]
                self.robot.state[2]=theta
                return True
            else:
                return False
        
    # ===================== EKF CORE =====================

    def predict(self, raw_drive_meas):
        F = self.state_transition(raw_drive_meas)

        # Advance robot kinematics
        self.robot.drive(raw_drive_meas)

        # ---- PREDICTION: tune via predict_covariance() & Q_STAB ----
        Q = self.predict_covariance(raw_drive_meas)
        # TIP:
        #  - If odom is too "stiff"/slow to correct -> increase Q_STAB (0.03–0.05)
        #  - If map becomes noisy -> decrease Q_STAB (0.01)
        self.P = F @ self.P @ F.T + Q

        # keep heading bounded
        x = self.get_state_vector()
        x[2,0] = self._wrap_angle(x[2,0])
        self.set_state_vector(x)

    def update(self, measurements):
        if not measurements:
            return

        # Build index list in your internal order
        tags = [lm.tag for lm in measurements]
        idx_list = [self.taglist.index(tag) for tag in tags]

        # Stack raw measurements (z) and covariance blocks (R)
        z = np.concatenate([lm.position.reshape(-1,1) for lm in measurements], axis=0)
        R = np.zeros((2*len(measurements),2*len(measurements)))
        for i in range(len(measurements)):
            R[2*i:2*i+2,2*i:2*i+2] = measurements[i].covariance

        # Predicted measurements and Jacobian
        z_hat = self.robot.measure(self.markers, idx_list).reshape((-1,1),order="F")
        H = self.robot.derivative_measure(self.markers, idx_list)

        x = self.get_state_vector()

        # --------- Gating + Adaptive-R per measurement ----------
        M = len(measurements)
        kept_rows = []
        R_adapt = np.zeros_like(R)

        for i in range(M):
            rr = slice(2*i, 2*i+2)
            z_i    = z[rr, :]
            zhat_i = z_hat[rr, :]
            H_i    = H[rr, :]

            # start with provided covariance (fallback: base)
            if hasattr(measurements[i], "covariance"):
                R_i = measurements[i].covariance
            else:
                R_i = np.diag([self.BASE_MEAS_STD_M**2, self.BASE_MEAS_STD_M**2])

            keep, d2 = self._gate_one(z_i, zhat_i, H_i, R_i)
            if keep:
                kept_rows.extend([2*i, 2*i+1])
                R_i_ad = self._adaptive_R(z_i, d2)
                R_adapt[rr, rr] = R_i_ad
            # If rejected, we simply don't include it in the stacked system

        if len(kept_rows) == 0:
            # Nothing to update this frame (all rejected)
            self.set_state_vector(x)
            return

        # Reduce to used rows
        z_used    = z[kept_rows, :]
        zhat_used = z_hat[kept_rows, :]
        H_used    = H[kept_rows, :]
        R_used    = R_adapt[np.ix_(kept_rows, kept_rows)]

        # EKF update (Joseph form; numerically stable)
        S = H_used @ self.P @ H_used.T + R_used
        K = np.linalg.solve(S.T, (H_used @ self.P).T).T

        innovation = z_used - zhat_used
        x = x + K @ innovation
        x[2,0] = self._wrap_angle(x[2,0])

        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H_used) @ self.P @ (I - K @ H_used).T + K @ R_used @ K.T

        self.set_state_vector(x)

    # ---------------------------------------------------------------

    def state_transition(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        F = np.eye(n)
        F[0:3,0:3] = self.robot.derivative_drive(raw_drive_meas)
        return F
    
    def predict_covariance(self, raw_drive_meas):
        """
        You currently map wheel noise -> pose covariance via robot.covariance_drive
        then add Q_STAB * I for stability.
        Tuning: raise Q_STAB if odom is too confident; lower if map gets noisy.
        """
        n = self.number_landmarks()*2 + 3
        Q = np.zeros((n,n))
        Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas) + self.Q_STAB*np.eye(3)
        return Q

    def add_landmarks(self, measurements):
        if not measurements:
            return

        th = self.robot.state[2]
        robot_xy = self.robot.state[0:2,:]
        R_theta = np.block([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])

        for lm in measurements:
            if lm.tag in self.taglist:
                continue
            
            lm_bff = lm.position
            lm_inertial = robot_xy + R_theta @ lm_bff

            # OPTIONAL: align world to external GT frame
            if self.eval_R is not None:
                lm_inertial = self.eval_R @ lm_inertial + self.eval_t

            self.taglist.append(int(lm.tag))
            self.markers = np.concatenate((self.markers, lm_inertial), axis=1)

            # Expand P by 2x2 block for the new landmark
            oldP = self.P
            self.P = np.zeros((oldP.shape[0]+2, oldP.shape[1]+2))
            self.P[:oldP.shape[0], :oldP.shape[1]] = oldP

            if self.USE_SMART_LM_INIT and hasattr(lm, "covariance"):
                # Initialize from current measurement covariance rotated into world
                R_bff = lm.covariance
                R_world = R_theta @ R_bff @ R_theta.T
                self.P[-2:, -2:] = 1.5 * R_world  # conservative 1.5x
            else:
                # Fallback: fixed variance from INIT_LM_STD_M
                s2 = self.INIT_LM_STD_M**2
                self.P[-2:, -2:] = np.diag([s2, s2])

    # =================== utilities from your file ===================

    @staticmethod
    def umeyama(from_points, to_points):
        assert len(from_points.shape) == 2, "from_points must be a m x n array"
        assert from_points.shape == to_points.shape, "shapes must match"
        N = from_points.shape[1]; m = 2
        mean_from = from_points.mean(axis = 1).reshape((2,1))
        mean_to   = to_points.mean(axis = 1).reshape((2,1))
        delta_from = from_points - mean_from
        delta_to   = to_points - mean_to
        cov_matrix = delta_to @ delta_from.T / N
        U, d, V_t = np.linalg.svd(cov_matrix, full_matrices = True)
        cov_rank = np.linalg.matrix_rank(cov_matrix)
        S = np.eye(m)
        if cov_rank >= m - 1 and np.linalg.det(cov_matrix) < 0:
            S[m-1, m-1] = -1
        elif cov_rank < m-1:
            raise ValueError(f"colinearity in covariance:\n{cov_matrix}")
        R = U.dot(S).dot(V_t)
        t = mean_to - R.dot(mean_from)
        return R, t

    # (draw functions unchanged)
    @ staticmethod
    def to_im_coor(xy, res, m2pixel):
        w, h = res
        x, y = xy
        x_im = int(-x*m2pixel+w/2.0)
        y_im = int(y*m2pixel+h/2.0)
        return (x_im, y_im)

    def draw_slam_state(self, res = (320, 500), not_pause=True):
        m2pixel = 100
        bg_rgb = np.array([213, 213, 213]).reshape(1, 1, 3) if not_pause else np.array([120, 120, 120]).reshape(1, 1, 3)
        canvas = np.ones((res[1], res[0], 3))*bg_rgb.astype(np.uint8)
        lms_xy = self.markers[:2, :]
        robot_xy = self.robot.state[:2, 0].reshape((2, 1))
        lms_xy = lms_xy - robot_xy
        robot_xy = robot_xy*0
        robot_theta = self.robot.state[2,0]
        start_point_uv = self.to_im_coor((0, 0), res, m2pixel)
        p_robot = self.P[0:2,0:2]
        axes_len,angle = self.make_ellipse(p_robot)
        canvas = cv2.ellipse(canvas, start_point_uv, 
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    angle, 0, 360, (0, 30, 56), 1)
        if self.number_landmarks() > 0:
            for i in range(len(self.markers[0,:])):
                xy = (lms_xy[0, i], lms_xy[1, i])
                coor_ = self.to_im_coor(xy, res, m2pixel)
                Plmi = self.P[3+2*i:3+2*(i+1),3+2*i:3+2*(i+1)]
                axes_len, angle = self.make_ellipse(Plmi)
                canvas = cv2.ellipse(canvas, coor_, 
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    angle, 0, 360, (244, 69, 96), 1)

        surface = pygame.surfarray.make_surface(np.rot90(canvas))
        surface = pygame.transform.flip(surface, True, False)
        surface.blit(self.rot_center(self.pibot_pic, robot_theta*57.3),
                    (start_point_uv[0]-15, start_point_uv[1]-15))
        if self.number_landmarks() > 0:
            for i in range(len(self.markers[0,:])):
                xy = (lms_xy[0, i], lms_xy[1, i])
                coor_ = self.to_im_coor(xy, res, m2pixel)
                try:
                    surface.blit(self.lm_pics[self.taglist[i]-1],
                    (coor_[0]-5, coor_[1]-5))
                except IndexError:
                    surface.blit(self.lm_pics[-1],
                    (coor_[0]-5, coor_[1]-5))
        return surface

    @staticmethod
    def rot_center(image, angle):
        orig_rect = image.get_rect()
        rot_image = pygame.transform.rotate(image, angle)
        rot_rect = orig_rect.copy()
        rot_rect.center = rot_image.get_rect().center
        rot_image = rot_image.subsurface(rot_rect).copy()
        return rot_image       

    @staticmethod
    def make_ellipse(P):
        e_vals, e_vecs = np.linalg.eig(P)
        idx = e_vals.argsort()[::-1]
        e_vals = e_vals[idx]
        e_vecs = e_vecs[:, idx]
        alpha = np.sqrt(4.605)
        axes_len = e_vals*2*alpha
        if abs(e_vecs[1, 0]) > 1e-3:
            angle = np.arctan(e_vecs[0, 0]/e_vecs[1, 0])
        else:
            angle = 0
        return (axes_len[0], axes_len[1]), angle
