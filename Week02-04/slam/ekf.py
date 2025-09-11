import numpy as np
from mapping_utils import MappingUtils
import cv2
import math
import pygame

class EKF:
    """
    EKF for landmark-based SLAM.
    State order: [x; y; theta; l1x; l1y; ...; lnx; lny]

    ── QUICK TUNING GUIDE ─────────────────────────────────────────────
    * BASE_MEAS_STD_M (R base): 0.03 m default. ↑ if detector noisy; ↓ if stable.
    * GATE_CHI2: 5.991 (95% in 2D). ↑ if rejecting too many good points.
    * RANGE_INFLATE: 0.8. Controls how R grows with distance (soft).
    * Q_STAB: 0.02. ↑ if odom is over-confident; ↓ if map too noisy.
    * INIT_LM_STD_M: 0.40 m if not using smart init.
    * USE_SMART_LM_INIT: True → init LM covariance from measurement.
    * ANCHOR_*: softened + delayed to avoid baking-in initial pose error.
    * MAX_RANGE_M: hard range gate. FAR_REJECT=True → drop far points.
    ───────────────────────────────────────────────────────────────────
    """

    # ===================== TUNABLES =====================
    BASE_MEAS_STD_M = 0.02     # base per-axis std for R (m)
    GATE_CHI2       = 5.991    # 95% chi^2 in 2D
    RANGE_INFLATE   = 3.45     # distance-based inflation strength (soft)
    Q_STAB          = 0.02     # process noise stabilizer on pose
    INIT_LM_STD_M   = 0.40     # fixed LM init std if not smart init
    USE_SMART_LM_INIT   = True

    # --- Anchor strategy ---
    ANCHOR_USE              = True
    ANCHOR_DELAY_MIN_LMS    = 2      # wait until ≥ this many landmarks seen
    ANCHOR_LM_STD_M         = 0.25   # ~25 cm soft anchor
    ANCHOR_MEAS_STD_M       = 0.10   # ~10 cm measurement std

    # --- Range management ---
    MAX_RANGE_M     = 1.6
    FAR_REJECT      = True
    FAR_R_MULT      = 3.0

    # Optional evaluation alignment (leave False unless you want to bake in GT)
    OPTIONAL_ALIGNMENT = False
    ALIGN_ROT_RAD   = -1.7354400353556125
    ALIGN_TX        = 0.2689604391944955
    ALIGN_TY        = 0.11240062133992615
    # ===================================================

    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def __init__(self, robot):
        # State
        self.robot = robot
        self.markers = np.zeros((2,0))
        self.taglist = []
        self.anchor_tag = None   # NEW: store the anchor tag id

        # Covariance (robot only at start)
        self.P = np.zeros((3,3))
        self.robot_init_state = None

        # UI assets
        self.lm_pics = []
        for i in range(1, 11):
            f_ = f'./pics/8bit/lm_{i}.png'
            self.lm_pics.append(pygame.image.load(f_))
        f_ = f'./pics/8bit/lm_unknown.png'
        self.lm_pics.append(pygame.image.load(f_))
        self.pibot_pic = pygame.image.load(f'./pics/8bit/pibot_top.png')

        # Optional alignment
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
        self.anchor_tag = None
        self.P = np.zeros((3,3))

    def number_landmarks(self):
        return int(self.markers.shape[1])

    def get_state_vector(self):
        return np.concatenate((self.robot.state, np.reshape(self.markers, (-1,1), order='F')), axis=0)

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
        lm_new = np.zeros((2,0))
        lm_prev = np.zeros((2,0))
        tag = []
        for lm in measurements:
            if lm.tag in self.taglist:
                lm_new  = np.concatenate((lm_new,  lm.position), axis=1)
                tag.append(int(lm.tag))
                lm_idx  = self.taglist.index(lm.tag)
                lm_prev = np.concatenate((lm_prev, self.markers[:,lm_idx].reshape(2,1)), axis=1)
        if int(lm_new.shape[1]) > 2:
            R,t = self.umeyama(lm_new, lm_prev)
            theta = math.atan2(R[1][0], R[0][0])
            self.robot.state[:2] = t[:2]
            self.robot.state[2]  = theta
            return True
        return False

    # ===================== EKF CORE =====================

    def predict(self, raw_drive_meas): #prediction is also state of the model that include both robot and landmarks that will need to be corrected with the measurement
        F = self.state_transition(raw_drive_meas)
        self.robot.drive(raw_drive_meas)
        Q = self.predict_covariance(raw_drive_meas)
        self.P = F @ self.P @ F.T + Q
        x = self.get_state_vector()
        x[2,0] = self._wrap_angle(x[2,0])
        self.set_state_vector(x)

    def _adaptive_R(self, z_i, d2=None, is_anchor=False, is_far=False):
        """
        Build 2x2 measurement covariance.
        - Anchor: soft, ignores inflations.
        - Normal: base + range inflation + residual inflation.
        - Far: optionally multiply by FAR_R_MULT if not rejecting.
        """
        if is_anchor:
            s = self.ANCHOR_MEAS_STD_M
            Rb = np.diag([s*s, s*s])
            return Rb

        s = self.BASE_MEAS_STD_M
        Rb = np.diag([s*s, s*s])

        # distance inflation
        r = float(np.linalg.norm(z_i))
        scale_r   = 1.0 + self.RANGE_INFLATE * (min(r, 2.0)/0.5)**2
        # residual inflation
        scale_res = 1.0 if d2 is None else min(3.0, 0.5 + d2/self.GATE_CHI2)

        R = (scale_r * scale_res) * Rb
        if is_far and not self.FAR_REJECT:
            R *= self.FAR_R_MULT
        return R

    def _gate_one(self, z_i, zhat_i, H_i, R_i):
        S_i = H_i @ self.P @ H_i.T + R_i
        v_i = z_i - zhat_i
        d2  = float(v_i.T @ np.linalg.solve(S_i, v_i))
        return (d2 <= self.GATE_CHI2), d2

    def update(self, measurements):
        if not measurements:
            return

        tags = [lm.tag for lm in measurements]
        idx_list = [self.taglist.index(tag) for tag in tags]

        z = np.concatenate([lm.position.reshape(-1,1) for lm in measurements], axis=0)
        R_stack = np.zeros((2*len(measurements),2*len(measurements)))
        for i in range(len(measurements)):
            if hasattr(measurements[i], "covariance"):
                R_stack[2*i:2*i+2,2*i:2*i+2] = measurements[i].covariance
            else:
                s = self.BASE_MEAS_STD_M
                R_stack[2*i:2*i+2,2*i:2*i+2] = np.diag([s*s, s*s])

        z_hat = self.robot.measure(self.markers, idx_list).reshape((-1,1), order="F")
        H     = self.robot.derivative_measure(self.markers, idx_list)

        x = self.get_state_vector()

        kept_rows = []
        R_adapt = np.zeros_like(R_stack)
        for i, tag in enumerate(tags):
            rr = slice(2*i, 2*i+2)
            z_i    = z[rr, :]
            zhat_i = z_hat[rr, :]
            H_i    = H[rr, :]

            r = float(np.linalg.norm(z_i))
            is_far = (r > self.MAX_RANGE_M)
            if is_far and self.FAR_REJECT:
                continue

            if hasattr(measurements[i], "covariance"):
                R_i = measurements[i].covariance
            else:
                s = self.BASE_MEAS_STD_M
                R_i = np.diag([s*s, s*s])

            is_anchor = (self.anchor_tag is not None and int(tag) == int(self.anchor_tag))

            keep, d2 = self._gate_one(z_i, zhat_i, H_i, R_i)
            if not keep:
                continue

            kept_rows.extend([2*i, 2*i+1])
            R_i_ad = self._adaptive_R(z_i, d2=d2, is_anchor=is_anchor, is_far=is_far)
            R_adapt[rr, rr] = R_i_ad

        if not kept_rows:
            self.set_state_vector(x)
            return

        z_used    = z[kept_rows, :]
        zhat_used = z_hat[kept_rows, :]
        H_used    = H[kept_rows, :]
        R_used    = R_adapt[np.ix_(kept_rows, kept_rows)]

        S = H_used @ self.P @ H_used.T + R_used
        K = (self.P @ H_used.T) @ np.linalg.inv(S)

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
        n = self.number_landmarks()*2 + 3
        Q = np.zeros((n,n))
        scale = 1.5
        Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas)*scale + self.Q_STAB*np.eye(3)
        return Q

    def add_landmarks(self, measurements):
        """
        Adds unseen landmarks with full EKF-SLAM initialization.
        - Computes landmark covariance with robot–landmark cross terms.
        - Handles anchor: first trusted landmark with softened covariance.
        """
        if not measurements:
            return

        x, y, th = self.robot.state.flatten()
        R_theta = np.array([[np.cos(th), -np.sin(th)],
                            [np.sin(th),  np.cos(th)]])
        P_rr = self.P[0:3, 0:3]

        for lm in measurements:
            if lm.tag in self.taglist:
                continue

            z = lm.position.flatten()
            l_world = np.array([[x], [y]]) + R_theta @ z.reshape(2,1)

            self.taglist.append(int(lm.tag))
            self.markers = np.concatenate((self.markers, l_world), axis=1)

            J_x = np.array([
                [1, 0, -np.sin(th)*z[0] - np.cos(th)*z[1]],
                [0, 1,  np.cos(th)*z[0] - np.sin(th)*z[1]]
            ])
            J_z = R_theta

            if hasattr(lm, "covariance"):
                R_meas = lm.covariance
            else:
                s = self.BASE_MEAS_STD_M
                R_meas = np.diag([s*s, s*s])

            P_ll = J_x @ P_rr @ J_x.T + J_z @ R_meas @ J_z.T
            P_rl = P_rr @ J_x.T
            P_lr = J_x @ P_rr

            oldP = self.P
            n = oldP.shape[0]
            newP = np.zeros((n+2, n+2))
            newP[:n, :n] = oldP

            set_soft_anchor = (
                self.ANCHOR_USE and
                self.anchor_tag is None and
                self.number_landmarks() >= self.ANCHOR_DELAY_MIN_LMS
            )

            if set_soft_anchor:
                self.anchor_tag = int(lm.tag)
                s2 = self.ANCHOR_LM_STD_M**2
                newP[n:n+2, n:n+2] = np.diag([s2, s2])
            else:
                newP[n:n+2, n:n+2] = P_ll
                newP[n:n+2, :3]    = P_lr
                newP[:3, n:n+2]    = P_rl
                # cross-terms with other landmarks ~ 0

            self.P = newP

    # =================== utilities & drawing ===================

    @staticmethod
    def umeyama(from_points, to_points):
        assert len(from_points.shape) == 2
        assert from_points.shape == to_points.shape
        N = from_points.shape[1]; m = 2
        mean_from = from_points.mean(axis=1).reshape((2,1))
        mean_to   = to_points.mean(axis=1).reshape((2,1))
        delta_from = from_points - mean_from
        delta_to   = to_points - mean_to
        cov_matrix = delta_to @ delta_from.T / N
        U, d, V_t = np.linalg.svd(cov_matrix, full_matrices=True)
        cov_rank = np.linalg.matrix_rank(cov_matrix)
        S = np.eye(m)
        if cov_rank >= m - 1 and np.linalg.det(cov_matrix) < 0:
            S[m-1, m-1] = -1
        elif cov_rank < m-1:
            raise ValueError("colinearity in covariance matrix")
        R = U.dot(S).dot(V_t)
        t = mean_to - R.dot(mean_from)
        return R, t

    @ staticmethod
    def to_im_coor(xy, res, m2pixel):
        w, h = res
        x, y = xy
        x_im = int(-x*m2pixel+w/2.0)
        y_im = int(y*m2pixel+h/2.0)
        return (x_im, y_im)

    def draw_slam_state(self, res=(320, 500), not_pause=True):
        m2pixel = 100
        bg_rgb = np.array([213, 213, 213]).reshape(1, 1, 3) if not_pause else np.array([120, 120, 120]).reshape(1, 1, 3)
        canvas = np.ones((res[1], res[0], 3))*bg_rgb.astype(np.uint8)
        lms_xy = self.markers[:2, :]
        robot_xy = self.robot.state[:2, 0].reshape((2, 1))
        lms_xy = lms_xy - robot_xy
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
                axes_len, a = self.make_ellipse(Plmi)
                canvas = cv2.ellipse(canvas, coor_, 
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    a, 0, 360, (244, 69, 96), 1)

        surface = pygame.surfarray.make_surface(np.rot90(canvas))
        surface = pygame.transform.flip(surface, True, False)
        surface.blit(self.rot_center(self.pibot_pic, robot_theta*57.3),
                    (start_point_uv[0]-15, start_point_uv[1]-15))
        if self.number_landmarks() > 0:
            for i in range(len(self.markers[0,:])):
               