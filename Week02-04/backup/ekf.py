import numpy as np
from mapping_utils import MappingUtils
import cv2
import math
import pygame


import json, re

class EKF:
    # Implementation of an EKF for SLAM
    # The state is ordered as [x; y; theta; l1x; l1y; ...; lnx; lny]

    ##########################################
    # Utility
    # Add outlier rejection here
    ##########################################

    def __init__(self, robot):
        # State components
        self.robot = robot
        self.markers = np.zeros((2,0))
        self.taglist = []

        # Covariance matrix
        self.P = np.zeros((3,3))
        self.init_lm_cov = 1e3
        self.robot_init_state = None
        self.lm_pics = []
        for i in range(1, 11):
            f_ = f'./pics/8bit/lm_{i}.png'
            self.lm_pics.append(pygame.image.load(f_))
        f_ = f'./pics/8bit/lm_unknown.png'
        self.lm_pics.append(pygame.image.load(f_))
        self.pibot_pic = pygame.image.load(f'./pics/8bit/pibot_top.png')
        
    def reset(self):
        self.robot.state = np.zeros((3, 1))
        self.markers = np.zeros((2,0))
        self.taglist = []
        # Covariance matrix
        self.P = np.zeros((3,3))
        self.init_lm_cov = 1e3
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
        
    ##########################################
    # EKF functions
    # Tune your SLAM algorithm here
    # ########################################

    # the prediction step of EKF
    def predict(self, raw_drive_meas):

        F = self.state_transition(raw_drive_meas)
        x = self.get_state_vector()

        # TODO: add your codes here to complete the prediction step
        Q = self.predict_covariance(raw_drive_meas)

        self.P = F @ self.P @ F.T + Q
    # the update step of EKF
    def update(self, measurements):
        if not measurements:
            return

        # Construct measurement index list
        tags = [lm.tag for lm in measurements]
        idx_list = [self.taglist.index(tag) for tag in tags]

        # Stack measurements and set covariance
        z = np.concatenate([lm.position.reshape(-1,1) for lm in measurements], axis=0)
        R = np.zeros((2*len(measurements),2*len(measurements)))
        for i in range(len(measurements)):
            R[2*i:2*i+2,2*i:2*i+2] = measurements[i].covariance

        # Compute own measurements
        z_hat = self.robot.measure(self.markers, idx_list)
        z_hat = z_hat.reshape((-1,1),order="F")
        H = self.robot.derivative_measure(self.markers, idx_list)

        x = self.get_state_vector()

        # TODO: add your codes here to compute the updated x
        y = z - z_hat
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S) 
        x_upd = x + K @ y 
        self.set_state_vector(x_upd)
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T


    def state_transition(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        F = np.eye(n)
        F[0:3,0:3] = self.robot.derivative_drive(raw_drive_meas)
        return F
    
    def predict_covariance(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        Q = np.zeros((n,n))
        Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas)+ 0.01*np.eye(3)
        return Q

    def add_landmarks(self, measurements):
        if not measurements:
            return

        th = self.robot.state[2]
        robot_xy = self.robot.state[0:2,:]
        R_theta = np.block([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])

        # Add new landmarks to the state
        for lm in measurements:
            if lm.tag in self.taglist:
                # ignore known tags
                continue
            
            lm_bff = lm.position
            lm_inertial = robot_xy + R_theta @ lm_bff

            self.taglist.append(int(lm.tag))
            self.markers = np.concatenate((self.markers, lm_inertial), axis=1)

            # Create a simple, large covariance to be fixed by the update step
            self.P = np.concatenate((self.P, np.zeros((2, self.P.shape[1]))), axis=0)
            self.P = np.concatenate((self.P, np.zeros((self.P.shape[0], 2))), axis=1)
            self.P[-2,-2] = self.init_lm_cov**2
            self.P[-1,-1] = self.init_lm_cov**2

    ##########################################
    ##########################################
    ##########################################

    @staticmethod
    def umeyama(from_points, to_points):

    
        assert len(from_points.shape) == 2, \
            "from_points must be a m x n array"
        assert from_points.shape == to_points.shape, \
            "from_points and to_points must have the same shape"
        
        N = from_points.shape[1]
        m = 2
        
        mean_from = from_points.mean(axis = 1).reshape((2,1))
        mean_to = to_points.mean(axis = 1).reshape((2,1))
        
        delta_from = from_points - mean_from # N x m
        delta_to = to_points - mean_to       # N x m
        
        cov_matrix = delta_to @ delta_from.T / N
        
        U, d, V_t = np.linalg.svd(cov_matrix, full_matrices = True)
        cov_rank = np.linalg.matrix_rank(cov_matrix)
        S = np.eye(m)
        
        if cov_rank >= m - 1 and np.linalg.det(cov_matrix) < 0:
            S[m-1, m-1] = -1
        elif cov_rank < m-1:
            raise ValueError("colinearility detected in covariance matrix:\n{}".format(cov_matrix))
        
        R = U.dot(S).dot(V_t)
        t = mean_to - R.dot(mean_from)
    
        return R, t

    # Plotting functions
    # ------------------
    @ staticmethod
    def to_im_coor(xy, res, m2pixel):
        w, h = res
        x, y = xy
        x_im = int(-x*m2pixel+w/2.0)
        y_im = int(y*m2pixel+h/2.0)
        return (x_im, y_im)

    def draw_slam_state(self, res = (320, 500), not_pause=True):
        # Draw landmarks
        m2pixel = 100
        if not_pause:
            bg_rgb = np.array([213, 213, 213]).reshape(1, 1, 3)
        else:
            bg_rgb = np.array([120, 120, 120]).reshape(1, 1, 3)
        canvas = np.ones((res[1], res[0], 3))*bg_rgb.astype(np.uint8)
        # in meters, 
        lms_xy = self.markers[:2, :]
        robot_xy = self.robot.state[:2, 0].reshape((2, 1))
        lms_xy = lms_xy - robot_xy
        robot_xy = robot_xy*0
        robot_theta = self.robot.state[2,0]
        # plot robot
        start_point_uv = self.to_im_coor((0, 0), res, m2pixel)
        
        p_robot = self.P[0:2,0:2]
        axes_len,angle = self.make_ellipse(p_robot)
        canvas = cv2.ellipse(canvas, start_point_uv, 
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    angle, 0, 360, (0, 30, 56), 1)
        # draw landmards
        if self.number_landmarks() > 0:
            for i in range(len(self.markers[0,:])):
                xy = (lms_xy[0, i], lms_xy[1, i])
                coor_ = self.to_im_coor(xy, res, m2pixel)
                # plot covariance
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
        """rotate an image while keeping its center and size"""
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

 


    # ===== True map helpers & comparison drawing =====
    @staticmethod
    def load_true_map(path="TrueMap.txt"):
        """读取 TrueMap.txt，返回 {tag_int: np.array([[x],[y]])} 的字典"""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        tag_map = {}
        p = re.compile(r"aruco(\d+)_0$")
        for k, v in data.items():
            m = p.match(k)
            if m:
                tag = int(m.group(1))
                tag_map[tag] = np.array([[float(v["x"])], [float(v["y"])]], dtype=float)
        return tag_map  # 仅返回 aruco 的真值（水果不参与 EKF 标记配准）

    def _alignment_est_to_true(self, true_map):
        """
        用共同观测到的标签做 Umeyama 刚体配准（无缩放），求从 EKF 坐标 -> True 坐标的 (R, t)。
        若可用点 < 2，则返回 (None, None)。
        """
        common_tags = [t for t in self.taglist if t in true_map]
        if len(common_tags) < 2:
            return None, None
        # 组装点集：from_points 为 EKF 中的 landmarks；to_points 为真值
        est_cols = []
        true_cols = []
        for t in common_tags:
            i = self.taglist.index(t)
            est_cols.append(self.markers[:, i].reshape(2, 1))
            true_cols.append(true_map[t].reshape(2, 1))
        from_points = np.concatenate(est_cols, axis=1)  # 2xN
        to_points   = np.concatenate(true_cols, axis=1) # 2xN
        R, t = self.umeyama(from_points, to_points)     # est -> true
        return R, t

    def compute_alignment_errors(self, true_map):
        """
        返回 (rmse, per_tag_error_dict, n_used)。
        若 <2 个共同标签，rmse 返回 None。
        """
        R, t = self._alignment_est_to_true(true_map)
        common_tags = [t_ for t_ in self.taglist if t_ in true_map]
        if R is None:
            return None, {}, 0
        errs = {}
        sq = []
        for t_ in common_tags:
            i = self.taglist.index(t_)
            p_est = self.markers[:, i].reshape(2, 1)     # EKF
            p_true = true_map[t_].reshape(2, 1)          # TRUE
            p_est_in_true = R @ p_est + t                # 对齐到真值坐标系
            e = float(np.linalg.norm(p_est_in_true - p_true))
            errs[t_] = e
            sq.append(e*e)
        rmse = float(np.sqrt(np.mean(sq))) if sq else None
        return rmse, errs, len(common_tags)

    def draw_slam_state_with_truth(self, true_map, res=(320, 500), not_pause=True):
        """
        在原 SLAM 视图上叠加真值 landmarks、对齐后的 EKF landmarks 以及误差连线。
        """
        # 先画基础 SLAM 视图
        base_surface = self.draw_slam_state(res=res, not_pause=not_pause)

        # 将 base_surface 转回 numpy 以便用 cv2 叠加图形
        # 注意：draw_slam_state 里做了 rot90 和 flip，这里保持同样方向
        # 我们重做一个同尺寸画布并对齐坐标系
        m2pixel = 100
        w, h = res
        bg = pygame.surfarray.array3d(base_surface)
        bg = np.rot90(bg, k=3)  # 旋回与 draw_slam_state 相反方向
        bg = pygame.transform.flip(base_surface, True, False)
        # 更简单：重新生成一个空画布并再次绘制覆盖（避免方向错乱）
        canvas = np.rot90(pygame.surfarray.array3d(base_surface))  # 转回 cv2 方向
        canvas = np.ascontiguousarray(canvas)

        # 计算配准（est -> true）
        R, t = self._alignment_est_to_true(true_map)

        # 机器人位姿（亦对齐至真值系，用于相对坐标）
        robot_xy = self.robot.state[0:2, :].reshape(2, 1)
        if R is not None:
            robot_xy_true = R @ robot_xy + t
        else:
            robot_xy_true = robot_xy  # 无法配准时，仅平移对比

        # 叠加真值 landmarks（蓝色实心圆）
        for tag, p_true in true_map.items():
            # 转到以机器人为原点的平移视图（不旋转，和原 draw_slam_state 一致）
            rel = (p_true - robot_xy_true).reshape(2,)
            uv = EKF.to_im_coor(rel, res, m2pixel)
            cv2.circle(canvas, uv, 4, (60, 100, 255), -1)  # BGR

        # 叠加 EKF landmarks（红色空心圆），并与真值连线
        if self.number_landmarks() > 0:
            for i, tag in enumerate(self.taglist):
                p_est = self.markers[:, i].reshape(2, 1)
                # 若有配准，用配准后的点与真值比较与连线
                if R is not None and tag in true_map:
                    p_est_true = R @ p_est + t
                    rel_est = (p_est_true - robot_xy_true).reshape(2,)
                    rel_true = (true_map[tag] - robot_xy_true).reshape(2,)
                    uv_est = EKF.to_im_coor(rel_est, res, m2pixel)
                    uv_true = EKF.to_im_coor(rel_true, res, m2pixel)
                    # 误差连线
                    cv2.line(canvas, uv_est, uv_true, (0, 215, 255), 1)
                    # 估计点
                    cv2.circle(canvas, uv_est, 5, (0, 0, 255), 1)
                else:
                    # 无配准/无真值时，只画估计点（浅红）
                    rel = (p_est - robot_xy).reshape(2,)  # EKF 原系
                    uv = EKF.to_im_coor(rel, res, m2pixel)
                    cv2.circle(canvas, uv, 5, (80, 80, 200), 1)

        # 在左上角写 RMSE
        rmse, _, n_used = self.compute_alignment_errors(true_map)
        label = f"RMSE: {'--' if rmse is None else f'{rmse:.03f} m'}  (N={n_used})"
        cv2.putText(canvas, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 2, cv2.LINE_AA)
        cv2.putText(canvas, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 245, 245), 1, cv2.LINE_AA)

        # 转回 pygame Surface，方向与原 draw_slam_state 保持一致
        surface = pygame.surfarray.make_surface(np.rot90(canvas))
        surface = pygame.transform.flip(surface, True, False)
        return surface

    def print_pose_and_rmse(self, true_map):
        """在终端打印机器人当前位姿 + landmark 对齐 RMSE"""
        x, y, theta = self.robot.state.flatten()
        rmse, _, n = self.compute_alignment_errors(true_map)
        if rmse is None:
            print(f"[Pose] x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad | RMSE: -- (need >=2 common tags)")
        else:
            print(f"[Pose] x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad | RMSE: {rmse:.3f} m (N={n})")
