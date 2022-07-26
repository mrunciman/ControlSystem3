from cylmarker import load_data, keypoints
from cylmarker.pose_estimation import img_segmentation
from scipy.io import savemat
import glob
import cv2 as cv
import numpy as np
import os.path
import scipy.io
from scipy.spatial.transform import Rotation as R
import scipy.io
import math

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    # return np.array([x, y, z])
    return np.rad2deg([x, y, z])

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
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
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
def check_image(im, im_path):
    if im is None:
        print('Error opening the image {}'.format(im_path))
        exit()


def show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg):
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

    # cv.imshow("Segmentation | Blue: background | Red: foreground", im_copy)
    # cv.waitKey(0)

    return mask_marker_bg


def show_contours_and_lines_and_centroids(im, pttrn):
    blue = (255, 0, 0)
    yellow=(124,225,255)
    red = (0, 0, 255)
    for sqnc in pttrn.list_sqnc:
        if sqnc.sqnc_id != -1:
            """ draw contours """
            for kpt in sqnc.list_kpts:
                cntr = kpt.cntr
                cv.drawContours(im, [cntr], -1, blue, 1)
            """ draw line between first and last kpt """
            u_0, v_0 = sqnc.list_kpts[0].get_centre_uv()
            u_1, v_1 = sqnc.list_kpts[-1].get_centre_uv()
            im = cv.line(im, (int(u_0), int(v_0)), (int(u_1), int(v_1)), yellow, 2, cv.LINE_AA) # lines
            """ draw centroids """
            for kpt in sqnc.list_kpts:
                u, v = kpt.get_centre_uv()
                im = cv.circle(im, (int(round(u)), int(round(v))), radius=2, color=red, thickness=-1)
    cv.imshow("image", im)
    cv.waitKey(0)

def make_rot(angle,axis='z'):
    cost = np.cos(np.deg2rad(angle))
    sint = np.sin(np.deg2rad(angle))
    if axis =='z':
        rot = np.array([[cost, -sint, 0],
                    [sint, cost, 0],
                    [0, 0, 1]])
    if axis =='x':
        rot = np.array([[1, 0, 0],
                    [0, cost, -sint],
                    [0, sint, cost]])
    if axis =='y':
        rot = np.array([[cost, 0, sint],
                    [0, 1, 0],
                    [-sint, 0, cost]])
    return rot

def show_axis(im, rvecs, tvecs, cam_matrix, dist_coeff, length):
    # top_distance = 22.85
    # top_distance = 17.66
    axis = np.float32([[0, 0, 0], [length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)
    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, cam_matrix, dist_coeff)
    imgpts = np.rint(imgpts).astype(int)
    frame_centre = tuple(imgpts[0].ravel())

    thickness = 4
    im = cv.line(im, frame_centre, tuple(imgpts[3].ravel()), (0,0,255), thickness, cv.LINE_AA)#R 3 Z
    im = cv.line(im, frame_centre, tuple(imgpts[2].ravel()), (0,255,0), thickness, cv.LINE_AA)#G 2 Y
    im = cv.line(im, frame_centre, tuple(imgpts[1].ravel()), (255,0,0), thickness, cv.LINE_AA)#B 1 X

    # cv.imshow("image", im)
    # cv.waitKey(0)

    return im

def get_transf_inv(transf):
    # ref: https://math.stackexchange.com/questions/152462/inverse-of-transformation-matrix
    r = transf[0:3, 0:3]
    t = transf[0:3, 3]
    r_inv = np.transpose(r) # Orthogonal matrix so the inverse is its transpose
    t_new = np.matmul(-r_inv, t).reshape(3, 1)
    transf_inv = np.concatenate((r_inv, t_new), axis = 1)
    return transf_inv


def save_pts_info(im_path, pnts_3d_object, pnts_2d_image):
    filename = '{}.pts3d.txt'.format(im_path)
    np.savetxt(filename, (np.squeeze(pnts_3d_object)), fmt="%s", delimiter=',')
    filename = '{}.pts2d.txt'.format(im_path)
    np.savetxt(filename, (np.squeeze(pnts_2d_image)), fmt="%s", delimiter=',')


def save_pose(im_path, mat):
    filename = '{}.txt'.format(im_path)
    print("save to ",filename)
    np.savetxt(filename, (mat), fmt="%s", delimiter=',')

def save_pose_mat(im_path, mat):
    filename = '{}.mat'.format(im_path)
    savemat(filename, {'data': mat})

def save_pose_np(im_path, mat):
    # print(mat)
    filename = '{}.npy'.format(im_path)
    np.save(filename, mat)

def show_reproj_error_image(im, pts2d_filtered, pts2d_projected):
    red = (0, 0, 255)
    green = (0, 255, 0)
    for pt2d_d, pt2d_p in zip(pts2d_filtered[:,0], pts2d_projected[:,0]):
        pt2d_d = (int(round(pt2d_d[0])), int(round(pt2d_d[1])))
        pt2d_p = (int(round(pt2d_p[0])), int(round(pt2d_p[1])))
        im = cv.line(im, pt2d_d, pt2d_p, color=red, thickness=1, lineType=cv.LINE_AA)
        im = cv.circle(im, pt2d_d, radius=1, color=red, thickness=-1)
        im = cv.circle(im, pt2d_p, radius=1, color=green, thickness=-1)
    cv.imshow("image", im)
    cv.waitKey(0)


def get_reprojection_error(pts3d, rvec, tvec, inliers, cam_matrix, dist_coeff, pnts2d, show_reproj_error, im):
    """ This function calculates the reprojection error of the inlier points """
    # Filter the inlier points
    n_inliers, _ = inliers.shape
    pts3d_filtered = np.zeros((n_inliers, pts3d.shape[1], pts3d.shape[2]), dtype=np.float)
    pts2d_filtered = np.zeros((n_inliers, pnts2d.shape[1], pnts2d.shape[2]), dtype=np.float)
    for ind_new, ind_old in enumerate(inliers[:,0]):
        pts3d_filtered[ind_new] = pts3d[ind_old]
        pts2d_filtered[ind_new] = pnts2d[ind_old]
    # Project 3D points into the 2D image
    pts2d_projected, jacobian = cv.projectPoints(pts3d_filtered, rvec, tvec, cam_matrix, dist_coeff)
    # Compare projected 2D points with the detected 2D points
    ## First ensure that they have the same shape
    pnts2d_detected = np.reshape(pts2d_filtered, pts2d_projected.shape)
    if show_reproj_error:
        show_reproj_error_image(im, pnts2d_detected, pts2d_projected)
    se = (pts2d_projected - pnts2d_detected) ** 2
    sse = np.sum(se)
    reproj_error = np.sqrt(sse / n_inliers) # Using the same formula as in OpenCV's calibration documentation
    return reproj_error # in [pixels]


def draw_detected_and_projected_features(rvecs, tvecs, cam_matrix, dist_coeff, pttrn, im):
    for sqnc in pttrn.list_sqnc:
        if sqnc.sqnc_id != -1:
            """
                We will draw the detected and projected contours of each feature in a sequence.
            """
            for kpt in sqnc.list_kpts:
                # First, we draw the detected contour (in green)
                cntr = kpt.cntr
                #im = cv.drawContours(im, [cntr], -1, [0, 255, 0], -1)
                im = cv.drawContours(im, [cntr], -1, [0, 255, 0], 1)
                # Then, we calculate + draw the projected contour (in red)
                corners_3d = np.float32(kpt.xyz_corners).reshape(-1,3)
                imgpts, jac = cv.projectPoints(corners_3d, rvecs, tvecs, cam_matrix, dist_coeff)
                imgpts = np.asarray(imgpts, dtype=np.int32)
                #im = cv.fillPoly(im, [imgpts], [0, 0, 255])
                im = cv.polylines(im, [imgpts], True, [0, 0, 255], thickness=1, lineType=cv.LINE_AA)
    return im

def read_trans_mat(mat_path):
    mat = scipy.io.loadmat(mat_path)
    Homo = mat['data'][0]
    Trans = Homo[:3].reshape(3,1)
    Rot = Homo[3:]
    B_r = quaternion_rotation_matrix(Rot)
    B = np.concatenate((B_r, Trans),axis=1)
    end = np.array([[0,0,0,1]])
    B = np.concatenate((B, end),axis=0)
    return B


def estimate_poses(cam_calib_data, config_file_data, data_pttrn, data_marker):
    ## Load pattern data
    is_write = False
    sqnc_max_ind = len(data_pttrn) - 1
    sequence_length = len(data_pttrn['sequence_0']['code']) # TODO: hardcoded
    ## Load camera matrix and distortion coefficients
    cam_matrix = cam_calib_data['camera_matrix']['data']
    cam_matrix = np.reshape(cam_matrix, (3, 3))
    dist_coeff_data = cam_calib_data['dist_coeff']['data']
    dist_coeff_np = np.array(dist_coeff_data)
    # Go through each image and estimate pose
    img_dir_path = config_file_data['img_dir_path']
    img_format = config_file_data['img_format']
    img_paths = load_data.load_img_paths(img_dir_path+'images/', img_format)
    undetected_num = 0

    for im_path in img_paths:
        im = cv.imread(im_path, cv.IMREAD_COLOR)
        check_image(im, im_path) # check if image was sucessfully read
        """ Step I - Undistort the input image """
        dist_coeff = dist_coeff_np
        im = cv.undistort(im, cam_matrix, dist_coeff)
        dist_coeff = None # we don't need to undistort again

        """ Step II - Segment the marker and detect features """
        mask_marker_bg, mask_marker_fg = img_segmentation.marker_segmentation(im, config_file_data)
        
        if mask_marker_bg is None:
            print("no masker background")
            continue
        # Draw segmented background and foreground
        # show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg)
        """ Step III - Identify features """
        pttrn = keypoints.find_keypoints(im, mask_marker_fg, config_file_data, sqnc_max_ind, sequence_length, data_pttrn, data_marker)
        # Estimate pose
        if pttrn is not None:
            # Draw contours and lines (for visualization)
            # show_contours_and_lines_and_centroids(im, pttrn)
            pnts_3d_object, pnts_2d_image = pttrn.get_data_for_pnp_solver()
            #save_pts_info(im_path, pnts_3d_object, pnts_2d_image)
            """ Step IV - Estimate the marker's pose """
            # valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_EPNP)
            valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_SQPNP)
            if valid:
                # im = draw_detected_and_projected_features(rvec_pred, tvec_pred, cam_matrix, dist_coeff, pttrn, im)
                show_reproj_error = False #True
                reproj_error = get_reprojection_error(pnts_3d_object, rvec_pred, tvec_pred, inliers, cam_matrix, dist_coeff, pnts_2d_image, show_reproj_error, im)
                # Draw axis
                rmat_pred, _ = cv.Rodrigues(rvec_pred)
                rot_angle = rotationMatrixToEulerAngles(rmat_pred)
                axis_im = show_axis(im, rvec_pred, tvec_pred, cam_matrix, dist_coeff, 6)
                # Save solution
                if is_write:
                    rmat_pred, _ = cv.Rodrigues(rvec_pred)
                    transf = np.concatenate((rmat_pred, tvec_pred), axis = 1)
                    save_pose_mat(img_dir_path+"homo/{}".format(img_idx), transf)
                    save_pose_np(img_dir_path+"homo/{}".format(img_idx), transf)
        else:
            print("none pattern detected")
            undetected_num+=1
            # show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg)
    print("total num: {}  undetected num: {}".format(len(img_paths), undetected_num))

def estimate_poses_live(cam_calib_data, config_file_data, data_pttrn, data_marker, cam_index):
    ## Load pattern data
    sqnc_max_ind = len(data_pttrn) - 1
    sequence_length = len(data_pttrn['sequence_0']['code']) # TODO: hardcoded
    ## Load camera matrix and distortion coefficients
    cam_matrix = cam_calib_data['camera_matrix']['data']
    cam_matrix = np.reshape(cam_matrix, (3, 3))
    dist_coeff_data = cam_calib_data['dist_coeff']['data']
    dist_coeff_np = np.array(dist_coeff_data)
    # Go through each image and estimate pose

    video = cv.VideoCapture(cam_index)
    width, height = 1280, 720
    video.set(cv.CAP_PROP_FRAME_WIDTH, width)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, height)

    if not os.path.exists('pose_record/'):
        os.makedirs('pose_record/')

    start_record=False
    idx = 0

    while True:
        ret, im = video.read()

        if not ret:
            print("not receving signals")
            break

        dist_coeff = dist_coeff_np
        im = cv.undistort(im, cam_matrix, dist_coeff)
        dist_coeff = None # we don't need to undistort again

        """ Step II - Segment the marker and detect features """
        mask_marker_bg, mask_marker_fg = img_segmentation.marker_segmentation(im, config_file_data)
        
        if mask_marker_bg is None:
            print("no masker background")
            continue
        # Draw segmented background and foreground
        mask_marker_bg = show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg)
        """ Step III - Identify features """
        pttrn = keypoints.find_keypoints(im, mask_marker_fg, config_file_data, sqnc_max_ind, sequence_length, data_pttrn, data_marker)
        # Estimate pose
        if pttrn is not None:
            # Draw contours and lines (for visualization)
            # show_contours_and_lines_and_centroids(im, pttrn)
            pnts_3d_object, pnts_2d_image = pttrn.get_data_for_pnp_solver()
            #save_pts_info(im_path, pnts_3d_object, pnts_2d_image)
            """ Step IV - Estimate the marker's pose """
            # valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_EPNP)
            valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_SQPNP)
            if valid:
                im = show_axis(im, rvec_pred, tvec_pred, cam_matrix, dist_coeff, 6)
        else:
            # print("none pattern detected")
            pass

        im_copy = np.concatenate((mask_marker_bg, im), axis=1)
        im = cv.resize(im_copy, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
        cv.imshow('Estimated Pose', im)
        cv.waitKey(1)

        if start_record:
            cv.imwrite('pose_record/{}.png'.format(idx),im)
            idx+=1

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('r'):
            print("start_record")
            start_record = True

    video.release()
    cv.destroyAllWindows()
