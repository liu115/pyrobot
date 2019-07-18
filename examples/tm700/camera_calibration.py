import sys
sys.path.append('/home/test/pyrobot/src')

import cv2
import numpy as np
import open3d as o3d
from pyrobot.core import Robot



class CameraCalibration:

    def __init__(self, grid_w=7, grid_h=5):
        self.bot = Robot('tm700')

        self.grid_w = grid_w
        self.grid_h = grid_h

        self.obj_pts = np.zeros((grid_w * grid_h, 3), np.float32)
        self.obj_pts[:, :2] = np.mgrid[0:grid_w, 0:grid_h].T.reshape(-1, 2) * 0.02

        self.camera_P, self.camera_D, self.camera_K = self.bot.camera.get_intrinsics()


    def _get_rgb(self):

        rgb = self.bot.camera.get_rgb()
        rgb = rgb[:, :, ::-1]
        return rgb

    def _get_corners(self, img):
        ret, corners = cv2.findChessboardCorners(img, (self.grid_w, self.grid_h), None)
        if not ret:
            raise ValueError('Chessboard corners not found')
        else:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), criteria)

        return corners

    def calibrate(self):
        rgb = self._get_rgb()
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Image', gray)
        cv2.waitKey(5000)

        # try:
        corners = self._get_corners(gray)
        corners = np.squeeze(corners)

        print(corners.shape)
        print(self.obj_pts.shape)
        print(self.camera_K.shape)
        print(self.camera_D.shape)
        ret, rvec, tvec = cv2.solvePnP(self.obj_pts, corners, self.camera_K, self.camera_D)

        print(ret)
        print(rvec)
        print(tvec)

        return corners, rvec, tvec

    def visualize_corners(self, img, corners):
        img = img.astype(np.uint8)
        for corner in corners:
            cv2.circle(img, tuple(corner), 3, (255, 0, 0), -1)
        # cv2.drawChessboardCorners(img, (self.grid_w, self.grid_h), corners, ret)

        return img


    def visualize_camera_3d(self, img, rvec, tvec):
        img = np.array(img)
        img = o3d.geometry.Image(img)
        rvec = np.squeeze(rvec)
        tvec = np.squeeze(tvec)

        pcd = o3d.PointCloud()
        pts = np.zeros((100, 3))
        for i in range(100):
            pts[i, :] = tvec + np.random.rand(3) * 1e-3
            pcd.points = o3d.Vector3dVector(pts)

        obj_pcd = o3d.PointCloud()
        obj_pcd.points = o3d.Vector3dVector(self.obj_pts)
        o3d.visualization.draw_geometries([obj_pcd, pcd])


    def _reproject(self, rvec, tvec):
        img_pts, _ = cv2.projectPoints(self.obj_pts, rvec, tvec, self.camera_K, self.camera_D)
        img_pts = img_pts.reshape(-1, 2)
        return img_pts

    def reproject_error(self, img_pts1, img_pts2):
        assert img_pts1.shape == img_pts2.shape
        return cv2.norm(img_pts1, img_pts2, cv2.NORM_L2) / img_pts1.shape[0]


    def visualize_reproject(self, img, rvec, tvec):
        pts = self._reproject(rvec, tvec)
        for pt in pts:
           cv2.circle(img, tuple(pt), 3, (0, 255, 0), -1)
        return img

if __name__ == '__main__':

    cc = CameraCalibration()
    print(cc.obj_pts.shape)
    corners, rvec, tvec = cc.calibrate()
    proj_corners = cc._reproject(rvec, tvec)
    print(corners.shape)
    print(proj_corners.shape)
    print(cc.reproject_error(corners, proj_corners))
    print(cc.reproject_error(corners, corners))

    img = cc._get_rgb()
    img = cc.visualize_corners(img, corners)
    img = cc.visualize_reproject(img, rvec, tvec)
    cc.visualize_camera_3d(img, rvec, tvec)
    cv2.imwrite('corners_ueye.jpg', img)
    cv2.imshow('corners', img)
    cv2.waitKey(0)

    # cc.visualize_camera_3d(img, rvec, tvec)
