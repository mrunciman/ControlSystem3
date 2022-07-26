from cam_pose import PoseEstimator
import time

config_path = 'data_45mm'
pose_est = PoseEstimator(config_path)
pose_est.initialize()

for i in range(20):
    # pose_est.pose_estimate(True)
    homo = pose_est.send_pose()
    if homo is not None:
        print(homo)
    else:
        print("no pattern detected")
    time.sleep(0.5)

pose_est.stop()

