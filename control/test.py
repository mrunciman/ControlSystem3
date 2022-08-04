from visual_navigation.cam_pose import PoseEstimator
import time

# config_path = '/control/visual_navigation/data_45mm/'
config_path = 'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/visual_navigation/data_45short/'
pose_est = PoseEstimator(config_path)
pose_est.initialize()

for i in range(500):
    pose_est.pose_estimate(True)
    # homo = pose_est.send_pose()
    # if homo is not None:
    #     print(homo)
    # else:
    #     print("no pattern detected")
    time.sleep(0.1)

pose_est.stop()

