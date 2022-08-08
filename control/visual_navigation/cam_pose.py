from pickle import TRUE
from visual_navigation.cylmarker import load_data, save_data, keypoints
# from cylmarker.pose_estimation import pose_estimation
# from cylmarker.make_new_pattern_and_marker import create_new_pattern, create_new_marker
from visual_navigation.cylmarker.pose_estimation import img_segmentation
# import argparse
import cv2 as cv
from visual_navigation.VideoCap import CameraSource
import numpy as np
import time

def quat_to_rot_matrix(i, j, k, w): 
    #requires quaternions in  w, i, j, k order
    q0 = w
    q1 = i
    q2 = j
    q3 = k
        
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1 
    r01 = 2 * (q1 * q2 - q0 * q3) 
    r02 = 2 * (q1 * q3 + q0 * q2) 
        
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3) 
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1) 
    
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2) 
    r21 = 2 * (q2 * q3 + q0 * q1) 
    r22 = 2 * (q0 * q0 + q3 * q3) - 1 
    
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],\
                            [r10, r11, r12],\
                            [r20, r21, r22]])
    
    return rot_matrix

def homo(R,T):
    T = T.reshape(3,-1)
    homo = np.concatenate((R, T), axis = 1)
    ones = np.array([0,0,0,1]).reshape(1,4)
    homo = np.concatenate((homo, ones), axis = 0)
    return homo

def unpack_homo(homo):
    R = homo[:,:3][:3]
    t = homo[:,-1][:3]
    return R,t


def show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg, is_show=False):
    im_copy = im.copy()
    alpha = 0.4
    # First we show the background part only
    mask_marker_bg = cv.subtract(mask_marker_bg, mask_marker_fg)
    mask_bg_blue = np.zeros_like(im_copy)
    mask_bg_blue[:,:,0] = mask_marker_bg
    im_copy = cv.addWeighted(im_copy, 1.0, mask_bg_blue, alpha, 0)
    # Then we show the foreground part
    alpha = 0.7
    mask_fg_red = np.zeros_like(im_copy)
    mask_fg_red[:,:,2] = mask_marker_fg
    im_copy = cv.addWeighted(im_copy, 1.0, mask_fg_red, alpha, 0)

    mask_marker_bg = cv.cvtColor(mask_marker_bg,cv.COLOR_GRAY2RGB)
    mask_marker_fg = cv.cvtColor(mask_marker_fg,cv.COLOR_GRAY2RGB)
    im_copy = np.concatenate((mask_marker_bg, im), axis=1)

    im_copy = cv.resize(im_copy, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)

    if is_show:
        cv.imshow("Segmentation | Blue: background | Red: foreground", im_copy)
        cv.waitKey(1)

    return mask_marker_bg
def show_axis(im, rvecs, tvecs, cam_matrix, dist_coeff, length, is_show=False):
    axis = np.float32([[0, 0, 0], [length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)
    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, cam_matrix, dist_coeff)
    imgpts = np.rint(imgpts).astype(int)
    frame_centre = tuple(imgpts[0].ravel())

    thickness = 4
    im = cv.line(im, frame_centre, tuple(imgpts[3].ravel()), (0,0,255), thickness, cv.LINE_AA)#R 3 Z
    im = cv.line(im, frame_centre, tuple(imgpts[2].ravel()), (0,255,0), thickness, cv.LINE_AA)#G 2 Y
    im = cv.line(im, frame_centre, tuple(imgpts[1].ravel()), (255,0,0), thickness, cv.LINE_AA)#B 1 X

    if is_show:
        cv.imshow("image", im)
        cv.waitKey(1)

    return im


class PoseEstimator:
    def __init__(self, config_path):
        self.config_path = config_path
        self.M_TO_MM = 1000

    def initialize(self,from_matlab=True):
        self.config_file_data, cam_calib_data = load_data.load_config_and_cam_calib_data(self.config_path)
        self.data_pttrn, self.data_marker = load_data.load_pttrn_and_marker_data(self.config_path)
        self.camera = CameraSource(self.config_file_data['cam_idx'],self.config_file_data['width'],self.config_file_data['height'])
        self.camera.initialize()
        ## Load pattern data
        self.sqnc_max_ind = len(self.data_pttrn) - 1
        self.sequence_length = len(self.data_pttrn['sequence_0']['code']) # TODO: hardcoded
        ## Load camera matrix and distortion coefficients
        cam_matrix = cam_calib_data['camera_matrix']['data']
        self.cam_matrix = np.reshape(cam_matrix, (3, 3))
        dist_coeff_data = cam_calib_data['dist_coeff']['data']
        self.dist_coeff = np.array(dist_coeff_data)

        # translation_path = self.data_pttrn['translation_path']
        # translation_path =  'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/visual_navigation/data_45mm/data0721/'
        translation_path =  'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/visual_navigation/data_45short/data_0801/'
        # translation_path =  './data_45mm/data0721/'
        if from_matlab:
            from scipy import io
            mat_path = translation_path+'XY.mat'
            mat = io.loadmat(mat_path)
            self.X = mat['X1']
            self.Y = mat['Y1']
        else:
            XY = np.load(translation_path+'XY.npy',allow_pickle=True).item()
            self.X = XY['X']
            self.Y = XY['Y']

        # rotMatrixR = quat_to_rot_matrix(-0.006375742,0.008078535,-0.003986573,-0.999939084) # Previous value
        rotMatrixR = quat_to_rot_matrix(-0.011488591, 0.014160476, -0.001230368, -0.999832988)
        
        # posR=[-0.110181898,0.029305253,0.133816868] # Previous value
        posR=[-0.112173915, 0.02704691, 0.082323581]

        T_W_Rob = np.block([[rotMatrixR[0, :], self.M_TO_MM*posR[0]],\
                                [rotMatrixR[1, :], self.M_TO_MM*posR[1]],\
                                [rotMatrixR[2, :], self.M_TO_MM*posR[2]],\
                                [0, 0, 0, 1]])
        self.T_Rob_W = np.linalg.pinv(T_W_Rob)
        # print(T_W_Rob)

        
    def stop(self):
        self.camera.end()

    def pose_estimate(self,is_show=False):
        is_save = False
        im = self.camera.receive_img()
        # print(im.shape)
        im = cv.undistort(im, self.cam_matrix, self.dist_coeff)
        dist_coeff = None # we don't need to undistort again

        """ Step II - Segment the marker and detect features """
        mask_marker_bg, mask_marker_fg = img_segmentation.marker_segmentation(im, self.config_file_data)

        # Draw segmented background and foreground
        mask_marker_bg = show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg)
        """ Step III - Identify features """
        pttrn = keypoints.find_keypoints(im, mask_marker_fg, self.config_file_data, self.sqnc_max_ind, self.sequence_length, self.data_pttrn, self.data_marker)
        # Estimate pose
        if pttrn is not None:
            # Draw contours and lines (for visualization)
            # show_contours_and_lines_and_centroids(im, pttrn)
            pnts_3d_object, pnts_2d_image = pttrn.get_data_for_pnp_solver()
            #save_pts_info(im_path, pnts_3d_object, pnts_2d_image)
            """ Step IV - Estimate the marker's pose """
            valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, self.cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_EPNP)
            # valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_SQPNP)
            if valid:
                im = show_axis(im, rvec_pred, tvec_pred, self.cam_matrix, dist_coeff, 6, is_show)
                rmat_pred, _ = cv.Rodrigues(rvec_pred)
            return homo(rmat_pred,tvec_pred)
        else:
            # print(im.shape)
            print("no pattern detected")
            if is_show:
                cv.imshow("image", mask_marker_bg)
                # cv.imshow("image", im)
                cv.waitKey(1)
            if is_save:
                cv.imwrite("nodetect.png".format(time.time()),im)
            return None
    def send_pose(self):
        h1 = self.pose_estimate()# Pattern to Camera
        if h1 is not None:
            temp1 = np.dot(np.dot(self.Y,h1),np.linalg.inv(self.X))
            R_temp, T_temp = unpack_homo(temp1)
            # trans_bias = np.array([0.36454775,0.70371892,-0.80012906])
            trans_bias = np.array([0.0,0.0,0.0])
            T_temp = T_temp+trans_bias
            temp1 = homo(R_temp, T_temp)
            return temp1# Estimated World to Instrument Pose
        else:
            return None
    def tip_pose(self):
        # Method to get tip pose with respect to robot base
        T_W_Inst = self.send_pose()# Estimated World to Instrument Pose
        # print("T_W_Inst: ",T_W_Inst)
        if T_W_Inst is not None:
            T_Rob_Inst = np.dot(self.T_Rob_W , T_W_Inst)
            print("T_Rob_Inst: ", T_Rob_Inst)
            return T_Rob_Inst
        else:
            return None
    

