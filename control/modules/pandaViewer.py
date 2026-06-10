from direct.showbase.ShowBase import ShowBase
from panda3d.core import LineSegs, NodePath, WindowProperties, LMatrix4f, PointLight, Texture, Material, TextureStage, DirectionalLight, AmbientLight
import math as mt
import numpy as np


class RobotViewer(ShowBase):
    def __init__(self, inputList):
        ShowBase.__init__(self)
        props = WindowProperties()
        props.setTitle("Motion Control")
        self.win.requestProperties(props)

        # Store input reference (list from Tkinter/shared memory)
        self.inputList = inputList
        # self.jointSpace = None
        self.lastJointSpace = [None, None, None, None, None, None]
        self.lastAngles = [None, None]
        self.lastPrismatic = None

        # Shaft parameters
        self.cylModelLength = 30
        self.shaftLength = 30
        self.shaftRadius = 1.5
        self.leverBaseZ = 10

        # Make shaft and add local axes
        self.RobotAssembly = NodePath("RobotAssembly")
        self.robotShaft = self.make_cylinder(self.shaftRadius, self.shaftLength)
        self.robotShaft.reparentTo(self.RobotAssembly)
        self.axes_local = self.make_axes(length=10)
        self.axes_local.setLightOff(1)
        self.axes_local.reparentTo(self.RobotAssembly)
        self.RobotAssembly.reparentTo(self.render)

        # Make shaft up to wrist
        self.UpToWrist = NodePath("UpToWrist")
        self.upToWrist = self.make_cylinder(self.shaftRadius, self.shaftLength)
        self.upToWrist.reparentTo(self.UpToWrist)
        self.UpToWrist.reparentTo(self.render)

        self.RobotWrist = NodePath("RobotWrist")
        self.axes_local = self.make_axes(length=10)
        self.axes_local.setLightOff(1)
        self.axes_local.reparentTo(self.RobotWrist)
        self.RobotWrist.reparentTo(self.render)

        # Global axes
        self.axes_global = self.make_axes(length=10)
        self.axes_global.setLightOff(1)
        self.axes_global.reparentTo(self.render)

        self.tranMatrix = self.RobotAssembly.get_mat(self.axes_global)

        # Camera setup
        self.setBackgroundColor(0.0, 0.0, 0.0, 1)
        self.cam.setPos(0, 0, 300)
        self.cam.lookAt(0,0,0)

        # Add a light to the scene
        # plight = PointLight('plight')
        # # plight.attenuation = (1, 0, 1)
        # plnp = self.render.attachNewNode(plight)
        # plnp.setPos(0, 100, 500)
        # self.render.setLight(plnp)

        dlight = DirectionalLight('dlight')
        dlight.setColor((1, 1, 1, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(0, 180, 0)
        self.render.setLight(dlnp)

        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.2, 0.2, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        # Update task
        self.taskMgr.add(self.check_input_gantry, "CheckInputTask")

    def make_cylinder(self, radius, height):
        """Use Panda3D's built-in geometry for a cylinder."""
        cyl = self.loader.loadModel("models/cylinder")
        tex = self.loader.loadTexture("models/LOGO_ROT.png")


        tex.setWrapU(Texture.WM_border_color)
        tex.setWrapV(Texture.WM_border_color)
        tex.setBorderColor((1.0, 1.0, 1.0, 1))

        ts = TextureStage("logo")
        cyl.setTexture(ts, tex)
        cyl.setTexScale(ts, 1/0.15, 1/0.2)
        cyl.setTexRotate(ts, 0)
        cyl.setTexOffset(ts, -1, -3.5)

        cyl_np = NodePath(cyl)
        return cyl_np

    def make_axes(self, length=1.0):
        """Draws X, Y, Z axes using colored lines."""
        lines = LineSegs()
        lines.setThickness(3.0)

        # X axis (red)
        lines.setColor(1, 0, 0, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(length, 0, 0)

        # Y axis (green)
        lines.setColor(0, 1, 0, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(0, length, 0)

        # Z axis (blue)
        lines.setColor(0, 0, 1, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(0, 0, length)

        node = lines.create()
        return NodePath(node)

    # def update_robot(self, angles, prismLen, shaftPosit):
    #     """Update robot joint positions based on angles and prismatic extension."""
    #     inclination = angles[0] 
    #     azimuth = -angles[1] 
    #     # print(inclination, azimuth, shaftPosit)

    #     # Scale cylinder along Z for prismatic extension
    #     scale_z = (self.shaftLength + prismLen)/self.cylModelLength
    #     self.robotShaft.setScale(1, 1, scale_z)

    #     tMatrixTrans = np.array([[1, 0, 0, shaftPosit[0]],\
    #                             [0, 1, 0, shaftPosit[1]],\
    #                             [0, 0, 1, self.leverBaseZ + shaftPosit[2]],\
    #                             [0, 0, 0, 1]])

    #     tMatrixRotX = np.array([[1, 0, 0, 0],\
    #                             [0, mt.cos(inclination), -mt.sin(inclination), 0],\
    #                             [0, mt.sin(inclination),  mt.cos(inclination), 0],\
    #                             [0, 0, 0, 1]])
        
    #     tMatrixRotY = np.array([[mt.cos(azimuth), 0, mt.sin(azimuth), 0],\
    #                             [ 0, 1, 0, 0],\
    #                             [-mt.sin(azimuth), 0, mt.cos(azimuth), 0],\
    #                             [0, 0, 0, 1]])
        
    #     # tMatrixTrans*tMatrixRotX*tMatrixRotY
    #     intermed1 = np.dot(tMatrixRotX, tMatrixTrans) 
    #     tMatrixRobot = np.dot(tMatrixRotY, intermed1)
    #     listOfLists = np.transpose(tMatrixRobot).tolist()
    #     # print(np.transpose(tMatrixRobot).tolist())
    #     flat_list = [x for xs in listOfLists for x in xs]
    #     self.tranMatrix = LMatrix4f(*flat_list)
        
    #     # Apply transform to assembly (position + rotations)
    #     self.RobotAssembly.set_mat(self.tranMatrix)


    def update_robot_gantry(self, jointSpace):
        """Update robot joint positions based on angles and prismatic extension."""

        alpha0 = 0
        dist0 = 0
        theta0 = 0
        prism0 = jointSpace[0] # Axial extension        

        alpha1 = 0
        dist1 = 0
        theta1 = mt.radians(jointSpace[1]) # Rotary angle
        prism1 = 0 # Axial extension

        alpha2 = -mt.pi/2
        dist2 = 0
        theta2 = -mt.pi/2   # Intermediate 1
        prism2 = 0

        alpha3 = 0
        dist3 = 0
        theta3 = mt.radians(jointSpace[3]) +  mt.pi/2  # Wrist angle
        prism3 = 0

        alpha4 = 0
        dist4 = 0
        theta4 = mt.pi/2    # Intermediate 2
        prism4 = 0

        alpha5 = mt.pi/2
        dist5 = 0
        theta5 = 0
        prism5 = 0

        alpha6 = 0
        dist6 = 0
        theta6 = 0
        prism6 = jointSpace[2] # Tool extension

        grasp = jointSpace[4]  # Grapser position

        # # --- Compute all intermediate steps natively on the GPU/C++ layer ---
        # # Pass any custom, changing variables right into the function arguments
        T_0_1 = self.make_dh_matrix(theta0, alpha0, dist0, prism0)
        T_1_2 = self.make_dh_matrix(theta1, alpha1, dist1, prism1)
        T_2_3 = self.make_dh_matrix(theta2, alpha2, dist2, prism2)
        T_3_4 = self.make_dh_matrix(theta3, alpha3, dist3, prism3)
        T_4_5 = self.make_dh_matrix(theta4, alpha4, dist4, prism4)
        T_5_6 = self.make_dh_matrix(theta5, alpha5, dist5, prism5)
        T_6_7 = self.make_dh_matrix(theta6, alpha6, dist6, prism6)

        # # Pure C++ matrix multiplications (Incredibly fast)
        T_0_2 = T_1_2 * T_0_1
        T_0_6 = T_5_6 * T_4_5 * T_3_4 * T_2_3 * T_0_2
        T_0_7 = T_6_7 * T_0_6 

       
        # Apply transform to assembly (position + rotations)
        self.UpToWrist.set_mat(T_0_2)

        # Apply transform to assembly (position + rotations)
        self.RobotWrist.set_mat(T_0_6)
        
        # Apply transform to assembly (position + rotations)
        self.RobotAssembly.set_mat(T_0_7)

        # Scale cylinder along Z up to the wrist
        scale_z_axial = (self.shaftLength + prism0)/self.cylModelLength
        self.upToWrist.setScale(1, 1, 1)

        # Scale cylinder along Z for the tool extension
        scale_z_tool = (self.shaftLength + prism5)/self.cylModelLength
        self.robotShaft.setScale(1, 1, 1)




        # # 1. Extract moving variables and evaluate trig functions once
        # prism0 = jointSpace[0]
        # prism6 = jointSpace[2]
        
        # theta1 = mt.radians(jointSpace[1])
        # theta3 = mt.radians(jointSpace[3]) + mt.pi/2
        
        # c1 = mt.cos(theta1)
        # s1 = mt.sin(theta1)
        # c3 = mt.cos(theta3)
        # s3 = mt.sin(theta3)

        # # Apply transform to UpToWrist (T_0_2 in Column-Major order)
        # self.tranMatrix_T_0_2 = LMatrix4f(
        #     c1, s1, 0, 0,
        #     -s1, c1, 0, 0,
        #     0, 0, 1, 0,
        #     0, 0, prism0, 1
        # )
        # self.UpToWrist.set_mat(self.tranMatrix_T_0_2)

        # # Apply transform to RobotWrist (T_0_6 in Column-Major order)
        # self.tranMatrix_T_0_6 = LMatrix4f(
        #     -s1*s3, c1*s3, -c3, 0,
        #     c1, s1, 0, 0,
        #     s1*c3, -c1*c3, -s3, 0,
        #     0, 0, prism0, 1
        # )
        # self.RobotWrist.set_mat(self.tranMatrix_T_0_6)

        # # Apply transform to RobotAssembly (T_0_7 in Column-Major order)
        # self.tranMatrix_T_0_7 = LMatrix4f(
        #     -s1*s3, c1*s3, -c3, 0,
        #     c1, s1, 0, 0,
        #     s1*c3, -c1*c3, -s3, 0,
        #     prism6*s1*c3, -prism6*c1*c3, prism0 - prism6*s3, 1
        # )
        # self.RobotAssembly.set_mat(self.tranMatrix_T_0_7)

        # print("C++ version: ", T_0_7)
        # print("Pre version: ", self.tranMatrix_T_0_7)

        # # Scale cylinders 
        # scale_z_axial = (self.shaftLength + prism0)/self.cylModelLength
        # self.upToWrist.setScale(1, 1, 1)

        # scale_z_tool = (self.shaftLength + 0)/self.cylModelLength
        # self.robotShaft.setScale(1, 1, 1)




    # def check_input(self, task):
    #     """Poll inputList for updates (simulating Tkinter shared state)."""
    #     try:
    #         anglesAndPosition = self.inputList
    #         angles = [anglesAndPosition[0], anglesAndPosition[1]]
    #         prism = anglesAndPosition[2]
    #         shaftPosit = [anglesAndPosition[3],
    #                       anglesAndPosition[4],
    #                       anglesAndPosition[5]]

    #         if ((angles != self.lastAngles) or (prism != self.lastPrismatic)):
    #             # print(angles)
    #             self.update_robot(angles, prism, shaftPosit)
    #             self.lastAngles = angles
    #             self.lastPrismatic = prism
    #     except Exception as e:
    #         print("Error:", e)

    #     return task.cont



    def check_input_gantry(self, task):
        """Poll inputList for updates (simulating Tkinter shared state)."""
        try:
            # localLastJointSpace = self.lastJointSpace
            localJointSpace = self.inputList

            # # print(localJointSpace[0])
            # # print(localLastJointSpace)
            # for l1, l2 in zip(localJointSpace, localLastJointSpace):
            #     if l1 != l2:
            #         self.lastJointSpace = localJointSpace
            #         # print("Update visualisation")
            #         # print("L1", l1)
            #         # print("L2", l2)
            #         break
            self.update_robot_gantry(localJointSpace)
        except Exception as e:
            print("Error:", e)

        return task.cont


    def make_dh_matrix(self, theta, alpha, dist, prism):
        """Generates a Panda3D-native row-vector DH matrix."""
        ct = mt.cos(theta)
        st = mt.sin(theta)
        ca = mt.cos(alpha)
        sa = mt.sin(alpha)
        
        # Constructed directly in transposed configuration
        return LMatrix4f(
            ct,      st,      0.0, 0.0,
            -st*ca,  ct*ca,   sa,  0.0,
            st*sa,  -ct*sa,   ca,  0.0,
            dist*ct, dist*st, prism, 1.0
        )




def run_viewer(inputList):
    app = RobotViewer(inputList)
    app.run()


if __name__ == "__main__":
    # Test with dummy values
    # Axial, rotary, extension, wrist, grasper
    run_viewer([50, 50, 20, 20, 0, 0])
