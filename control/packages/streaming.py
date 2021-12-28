import optirx as rx

class optiTracker:

    def __init__(self):
        self.marker1 = (0, 0, 0)
        self.marker2 = (0, 0, 0)
        self.marker3 = (0, 0, 0)
        self.version = (2, 10, 0, 0)  # NatNet version to use
        self.trackSock = rx.mkdatasock()
        self.packet = []
        self.trackData = []
    
    def readSocket(self):
        self.trackData = self.trackSock.recv(rx.MAX_PACKETSIZE)
        self.packet = rx.unpack(self.trackData, version=self.version)
        if type(self.packet) is rx.SenderData:
            self.version = self.packet.natnet_version
        self.marker1 = self.packet.labeled_markers[0].position
        self.marker2 = self.packet.labeled_markers[1].position
        self.marker3 = self.packet.labeled_markers[2].position
        print("M1: ", self.marker1, "M2: ", self.marker2, "M3: ", self.marker3)
        return self.marker1, self.marker2, self.marker3

    def closeSocket(self):
        self.trackSock.shutdown()
        self.trackSock.close()


# optiTrack = optiTracker()
# optiTrack.readSocket()

    # FrameOfData(frameno=146765, sets={b'all': []}, other_markers=[(0.09026086330413818, 0.2435210943222046, 0.015333890914916992),\
    #  (0.08072531968355179, 0.002592326607555151, -0.06814895570278168), (0.07142879068851471, -0.2369968295097351, -0.15248703956604004)],\
    #  rigid_bodies=[], skeletons=[], labeled_markers=[LabeledMarker(id=5028,\
    #  position=(0.09026086330413818, 0.2435210943222046, 0.015333890914916992),\
    #  size=0.011816475540399551, occluded=False, point_cloud_solved=True, model_solved=False),\
    #  LabeledMarker(id=5029, position=(0.08072531968355179, 0.002592326607555151, -0.06814895570278168),\
    #  size=0.012152580544352531, occluded=False, point_cloud_solved=True, model_solved=False),\
    #  LabeledMarker(id=5030, position=(0.07142879068851471, -0.2369968295097351, -0.15248703956604004),\
    #  size=0.013185441493988037, occluded=False, point_cloud_solved=True, model_solved=False)],\
    #  latency=0.5968000292778015, timecode=(0, 0), timestamp=1223.0416666666667, is_recording=False, tracked_models_changed=False)

# FrameOfData(frameno=253179, sets={b'all': []}, other_markers=[(0.090259850025177, 0.2434762716293335, 0.015366792678833008), (0.08073146641254425, 0.0025320868007838726, -0.0681605339050293), (0.0714418888092041, -0.23710662126541138, -0.15255498886108398)], rigid_bodies=[], skeletons=[], labeled_markers=[LabeledMarker(id=5028, position=(0.090259850025177, 0.2434762716293335, 0.015366792678833008), size=0.011802209541201591, occluded=False, point_cloud_solved=True, model_solved=False), LabeledMarker(id=5029, position=(0.08073146641254425, 0.0025320868007838726, -0.0681605339050293), size=0.0121090617030859, occluded=False, point_cloud_solved=True, model_solved=False), LabeledMarker(id=5030, position=(0.0714418888092041, -0.23710662126541138, -0.15255498886108398), size=0.013170532882213593, occluded=False, point_cloud_solved=True, model_solved=False)], latency=0.2362000048160553, timecode=(0, 0), timestamp=2109.825, is_recording=False, tracked_models_changed=False)
# [LabeledMarker(id=5028, position=(0.09026548266410828, 0.2434368133544922, 0.015464067459106445), size=0.011778432875871658, occluded=False, point_cloud_solved=True, model_solved=False), LabeledMarker(id=5029, position=(0.08076581358909607, 0.0025018169544637203, -0.06814271211624146), size=0.012079177424311638, occluded=False, point_cloud_solved=True, model_solved=False), LabeledMarker(id=5030, position=(0.0714956670999527, -0.23715823888778687, -0.15259003639221191), size=0.013163866475224495, occluded=False, point_cloud_solved=True, model_solved=False)]
# LabeledMarker(id=5029, position=(0.08076213300228119, 0.0024986625649034977, -0.06813696026802063), size=0.012080874294042587, occluded=False, point_cloud_solved=True, model_solved=False)


# For closed loop:

# Distinguish markers and calculate entry point positions
# Calculate distances between entry points and end effector (cable lengths)
# Smooth or filter incoming values
# Convert cable lengths to volumes (lookup tables)
# Convert volume to step number (stepper positions)
# Calculate error and new, controlled step number
# Send to arduinos