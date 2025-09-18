import numpy as np
from mapping_utils import MappingUtils
import cv2
import math
import pygame

class EKF:
    """
    EKF for landmark-based SLAM.
    State order: [x; y; theta; l1x; l1y; ...; lnx; lny]
    """

    # ===================== TUNABLES =====================
    BASE_MEAS_STD_M = 0.02
    GATE_CHI2       = 5.991      # 95% in 2D
    RANGE_INFLATE   = 3.45
    Q_STAB          = 0.02
    INIT_LM_STD_M   = 0.40
    USE_SMART_LM_INIT = True

    ANCHOR_USE        = True
    ANCHOR_LM_STD_M   = 0.01
    ANCHOR_MEAS_STD_M = 0.015

    MAX_RANGE_M   = 1.6
    FAR_REJECT    = True
    FAR_R_MULT    = 3.0

    OPTIONAL_ALIGNMENT = False
    ALIGN_ROT_RAD   = 0.0
    ALIGN_TX        = 0.0
    ALIGN_TY        = 0.0

    # ---- New quality gates ----
    GOOD_BEARING_DEG     = 25.0   # |bearing| <= 25° considered “front-on”
    STATIONARY_V_THRESH  = 0.02   # m/s
    STATIONARY_W_THRESH  = 0.05   # rad/s
    # ====================================================

    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def __init__(self, robot):
        self.robot = robot
        self.markers  = np.zeros((2,0))
        self.taglist  = []
        self.anchor_tag = None

        self.P = np.zeros((3,3))
        self.robot_init_state = None

        # optional eval alignment
        if self.OPTIONAL_ALIGNMENT:
            c, s = np.cos(self.ALIGN_ROT_RAD), np.sin(self.ALIGN_ROT_RAD)
            self.eval_R = np.array([[c, -s],[s, c]])
            self.eval_t = np.array([[self.ALIGN_TX],[self.ALIGN_TY]])
        else:
            self.eval_R = None
            self.eval_t = None

        # UI assets
        self.lm_pics = []
        for i in range(1, 11):
            self.lm_pics.append(pygame.image.load(f'./pics/8bit/lm_{i}.png'))
        self.lm_pics.append(pygame.image.load(f'./pics/8bit/lm_unknown.png'))
        self.pibot_pic = pygame.image.load(f'./pics/8bit/pibot_top.png')

        # diagnostics / HUD
        self.last_nis = None
        self.status = {"moving": False, "angle_good": False, "can_add": False}

    def reset(self):
        self.robot.state = np.zeros((3, 1))
        self.markers   = np.zeros((2,0))
        self.taglist   = []
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

    # ------------ Pause/Recover ------------
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

    def predict(self, drive_meas):
        lv = drive_meas.left_speed
        rv = drive_meas.right_speed
        dt = drive_meas.dt

        # if no wheel movement, skip
        if abs(lv) < 1e-6 and abs(rv) < 1e-6:
            return  

        F = self.state_transition(drive_meas)
        self.robot.drive(drive_meas)
        Q = self.predict_covariance(drive_meas)
        self.P = F @ self.P @ F.T + Q

        x = self.get_state_vector()
        x[2,0] = self._wrap_angle(x[2,0])
        self.set_state_vector(x)

    def _adaptive_R(self, z_i, d2=None, is_anchor=False, is_far=False):
        if is_anchor:
            s = self.ANCHOR_MEAS_STD_M
            return np.diag([s*s, s*s])

        s = self.BASE_MEAS_STD_M
        Rb = np.diag([s*s, s*s])

        r = float(np.linalg.norm(z_i))
        scale_r   = 1.0 + self.RANGE_INFLATE * (min(r, 2.0)/0.5)**2
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
        idx_list = [self.taglist.index(tag) for tag in tags if tag in self.taglist]
        if not idx_list:
            return

        # Only use measurements whose tags we already have in state
        kept_ms = [lm for lm in measurements if lm.tag in self.taglist]
        z = np.concatenate([lm.position.reshape(-1,1) for lm in kept_ms], axis=0)

        R_stack = np.zeros((2*len(kept_ms), 2*len(kept_ms)))
        for i, lm in enumerate(kept_ms):
            if hasattr(lm, "covariance"):
                R_stack[2*i:2*i+2, 2*i:2*i+2] = lm.covariance
            else:
                s = self.BASE_MEAS_STD_M
                R_stack[2*i:2*i+2, 2*i:2*i+2] = np.diag([s*s, s*s])

        z_hat = self.robot.measure(self.markers, idx_list).reshape((-1,1), order="F")
        H     = self.robot.derivative_measure(self.markers, idx_list)

        x = self.get_state_vector()

        kept_rows = []
        R_adapt   = np.zeros_like(R_stack)
        nis_terms = []

        for i, lm in enumerate(kept_ms):
            rr = slice(2*i, 2*i+2)
            z_i, zhat_i, H_i = z[rr,:], z_hat[rr,:], H[rr,:]
            r  = float(np.linalg.norm(z_i))
            far = (r > self.MAX_RANGE_M)

            if far and self.FAR_REJECT:
                continue

            keep, d2 = self._gate_one(z_i, zhat_i, H_i, R_stack[rr, rr])
            if not keep:
                continue

            kept_rows.extend([2*i, 2*i+1])
            is_anchor = (self.anchor_tag is not None and int(lm.tag) == int(self.anchor_tag))
            R_i_ad = self._adaptive_R(z_i, d2=d2, is_anchor=is_anchor, is_far=far)
            R_adapt[rr, rr] = R_i_ad
            nis_terms.append(d2)

        if not kept_rows:
            self.set_state_vector(x)
            self.last_nis = None
            return

        z_used    = z[kept_rows, :]
        zhat_used = z_hat[kept_rows, :]
        H_used    = H[kept_rows, :]
        R_used    = R_adapt[np.ix_(kept_rows, kept_rows)]

        S   = H_used @ self.P @ H_used.T + R_used          # (k x k)
        PHt = self.P @ H_used.T                            # (n x k)
        Y   = np.linalg.solve(S, PHt.T)                    # S Y = PHt^T
        K   = Y.T

        innovation = z_used - zhat_used
        x = x + K @ innovation
        x[2,0] = self._wrap_angle(x[2,0])

        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H_used) @ self.P @ (I - K @ H_used).T + K @ R_used @ K.T

        self.set_state_vector(x)
        self.last_nis = float(innovation.T @ np.linalg.solve(S, innovation))

    # -------- New: quality gating for *initialization* --------

    def _bearing_is_good(self, lm_bff):
        """lm_bff is 2x1 in body frame. Bearing = atan2(y,x)."""
        bearing = float(np.arctan2(lm_bff[1,0], lm_bff[0,0]))
        return abs(bearing) <= np.deg2rad(self.GOOD_BEARING_DEG)

    def add_landmarks(self, measurements, moving=False):
        """
        Only *initialize* new landmarks when (a) robot is stationary and (b) angle is good.
        Still updates existing landmarks in update(); this only gates *initialization*.
        Also updates self.status for the HUD.
        """
        if not measurements:
            self.status.update({"moving": bool(moving), "angle_good": False, "can_add": False})
            return

        th = float(self.robot.state[2])
        robot_xy = self.robot.state[0:2, :]
        R_theta  = np.block([[np.cos(th), -np.sin(th)],
                             [np.sin(th),  np.cos(th)]])

        any_good_angle = False

        for lm in measurements:
            if lm.tag in self.taglist:
                continue

            lm_bff = lm.position  # 2x1 body-frame
            is_good = self._bearing_is_good(lm_bff)
            any_good_angle = any_good_angle or is_good

            # only *initialize* when robot is stationary AND view angle is good
            if moving or (not is_good):
                continue

            lm_world = robot_xy + R_theta @ lm_bff
            if self.eval_R is not None:
                lm_world = self.eval_R @ lm_world + self.eval_t

            # add to state
            self.taglist.append(int(lm.tag))
            self.markers = np.concatenate((self.markers, lm_world), axis=1)

            # expand covariance
            oldP = self.P
            self.P = np.zeros((oldP.shape[0]+2, oldP.shape[1]+2))
            self.P[:oldP.shape[0], :oldP.shape[1]] = oldP

            # init landmark covariance
            if self.ANCHOR_USE and self.anchor_tag is None:
                self.anchor_tag = int(lm.tag)
                s2 = self.ANCHOR_LM_STD_M**2
                self.P[-2:, -2:] = np.diag([s2, s2])
            elif self.USE_SMART_LM_INIT and hasattr(lm, "covariance"):
                R_bff   = lm.covariance
                R_world = R_theta @ R_bff @ R_theta.T
                self.P[-2:, -2:] = 1.5 * R_world
            else:
                s2 = self.INIT_LM_STD_M**2
                self.P[-2:, -2:] = np.diag([s2, s2])

        can_add = (not moving) and any_good_angle
        self.status.update({"moving": bool(moving), "angle_good": bool(any_good_angle), "can_add": bool(can_add)})

    # ---------------------------------------------------------------

    def state_transition(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        F = np.eye(n)
        F[0:3,0:3] = self.robot.derivative_drive(raw_drive_meas)
        return F

    def predict_covariance(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        Q = np.zeros((n,n))
        scale = 1
        Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas)*scale + self.Q_STAB*np.eye(3)
        return Q

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

    @staticmethod
    def _cov_to_ellipse(P_2x2, scale_alpha=np.sqrt(5.991)):  # 95% to match gating
        # Return major/minor axes (meters) and heading (radians)
        e_vals, e_vecs = np.linalg.eig(P_2x2)
        idx = e_vals.argsort()[::-1]
        e_vals = e_vals[idx]
        e_vecs = e_vecs[:, idx]
        axes_len = np.sqrt(np.maximum(e_vals, 0.0)) * scale_alpha  # std -> conf radius
        angle    = math.atan2(e_vecs[1, 0], e_vecs[0, 0])          # robust orientation
        return (axes_len[0], axes_len[1]), angle

    @staticmethod
    def rot_center(image, angle):
        orig_rect = image.get_rect()
        rot_image = pygame.transform.rotate(image, angle*57.2957795)
        rot_rect  = orig_rect.copy()
        rot_rect.center = rot_image.get_rect().center
        rot_image = rot_image.subsurface(rot_rect).copy()
        return rot_image

    def draw_slam_state(self, res=(320, 500), not_pause=True):
        m2pixel = 100
        bg_rgb = np.array([213, 213, 213]).reshape(1, 1, 3) if not_pause else np.array([120, 120, 120]).reshape(1, 1, 3)
        canvas = np.ones((res[1], res[0], 3))*bg_rgb.astype(np.uint8)

        lms_xy = self.markers[:2, :]
        robot_xy = self.robot.state[:2, 0].reshape((2, 1))
        rel_xy = lms_xy - robot_xy
        robot_theta = float(self.robot.state[2])
        origin_uv = self.to_im_coor((0, 0), res, m2pixel)

        # robot covariance ellipse
        Prr = self.P[0:2,0:2]
        axes_len, angle = self._cov_to_ellipse(Prr)
        canvas = cv2.ellipse(canvas, origin_uv,
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    math.degrees(angle), 0, 360, (0, 30, 56), 1)

        # landmark ellipses
        for i in range(self.number_landmarks()):
            xy = (rel_xy[0, i], rel_xy[1, i])
            uv = self.to_im_coor(xy, res, m2pixel)
            Plmi = self.P[3+2*i:3+2*(i+1),3+2*i:3+2*(i+1)]
            axes_len_lm, ang_lm = self._cov_to_ellipse(Plmi)
            canvas = cv2.ellipse(canvas, uv,
                (int(axes_len_lm[0]*m2pixel), int(axes_len_lm[1]*m2pixel)),
                math.degrees(ang_lm), 0, 360, (244, 69, 96), 1)

        surface = pygame.surfarray.make_surface(np.rot90(canvas))
        surface = pygame.transform.flip(surface, True, False)
        surface.blit(self.rot_center(self.pibot_pic, robot_theta),
                    (origin_uv[0]-15, origin_uv[1]-15))

        for i in range(self.number_landmarks()):
            xy = (rel_xy[0, i], rel_xy[1, i])
            uv = self.to_im_coor(xy, res, m2pixel)
            try:
                surface.blit(self.lm_pics[self.taglist[i]-1], (uv[0]-5, uv[1]-5))
            except IndexError:
                surface.blit(self.lm_pics[-1], (uv[0]-5, uv[1]-5))
        return surface
