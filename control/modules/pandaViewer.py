from direct.showbase.ShowBase import ShowBase
from panda3d.core import LineSegs, NodePath, WindowProperties, LMatrix4f, PointLight, Texture, Material, TextureStage, DirectionalLight, AmbientLight
import math as mt
import numpy as np


class RobotViewer(ShowBase):
    def __init__(self, inputList):
        ShowBase.__init__(self)
        props = WindowProperties()
        props.setTitle("Robot Arm Viewer (Panda3D)")
        self.win.requestProperties(props)

        # Store input reference (list from Tkinter/shared memory)
        self.inputList = inputList
        self.jointSpace = None
        self.lastJointSpace = None
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

    def update_robot(self, angles, prismLen, shaftPosit):
        """Update robot joint positions based on angles and prismatic extension."""
        inclination = angles[0] 
        azimuth = -angles[1] 
        # print(inclination, azimuth, shaftPosit)

        # Scale cylinder along Z for prismatic extension
        scale_z = (self.shaftLength + prismLen)/self.cylModelLength
        self.robotShaft.setScale(1, 1, scale_z)

        tMatrixTrans = np.array([[1, 0, 0, shaftPosit[0]],\
                                [0, 1, 0, shaftPosit[1]],\
                                [0, 0, 1, self.leverBaseZ + shaftPosit[2]],\
                                [0, 0, 0, 1]])

        tMatrixRotX = np.array([[1, 0, 0, 0],\
                                [0, mt.cos(inclination), -mt.sin(inclination), 0],\
                                [0, mt.sin(inclination),  mt.cos(inclination), 0],\
                                [0, 0, 0, 1]])
        
        tMatrixRotY = np.array([[mt.cos(azimuth), 0, mt.sin(azimuth), 0],\
                                [ 0, 1, 0, 0],\
                                [-mt.sin(azimuth), 0, mt.cos(azimuth), 0],\
                                [0, 0, 0, 1]])
        
        # tMatrixTrans*tMatrixRotX*tMatrixRotY
        intermed1 = np.dot(tMatrixRotX, tMatrixTrans) 
        tMatrixRobot = np.dot(tMatrixRotY, intermed1)
        listOfLists = np.transpose(tMatrixRobot).tolist()
        # print(np.transpose(tMatrixRobot).tolist())
        flat_list = [x for xs in listOfLists for x in xs]
        self.tranMatrix = LMatrix4f(*flat_list)
        
        # Apply transform to assembly (position + rotations)
        self.RobotAssembly.set_mat(self.tranMatrix)


    def update_robot_gantry(self, jointSpace):
        """Update robot joint positions based on angles and prismatic extension."""
        theta0 = 0
        prism0 = jointSpace[0] # Axial extension
        alpha0 = 0
        dist0 = 0

        theta1 = mt.radians(jointSpace[1]) # Rotary angle
        prism1 = 0 # Axial extension
        alpha1 = -mt.pi/2
        dist1 = 0

        theta2 = -mt.pi/2   # Intermediate
        prism2 = 0
        alpha2 = 0
        dist2 = 0

        theta3 = mt.radians(jointSpace[3]) # Wrist angle
        prism3 = 0
        alpha3 = 0
        dist3 = 0

        theta4 = mt.pi/2    # Intermediate 2
        prism4 = 0
        alpha4 = mt.pi/2
        dist4 = 0

        theta5 = 0
        prism5 = jointSpace[2] # Tool extension
        alpha5 = 0
        dist5 = 0

        grasp = jointSpace[4]  # Grapser position

        # Scale cylinder along Z for prismatic extension
        scale_z = (self.shaftLength + prism5)/self.cylModelLength
        self.robotShaft.setScale(1, 1, scale_z)

      
        T_0_1 = np.array([  [mt.cos(alpha0), -mt.sin(theta0)*mt.cos(alpha0),  mt.sin(theta0)*mt.sin(alpha0), 0],\
                            [mt.sin(theta0),  mt.cos(theta0)*mt.cos(alpha0), -mt.cos(theta0)*mt.sin(alpha0), 0],\
                            [0,               mt.sin(alpha0),                 mt.cos(alpha0),                prism0],\
                            [0, 0, 0, 1]])
        print(T_0_1)
        
        T_1_2 = np.array([  [mt.cos(alpha1), -mt.sin(theta1)*mt.cos(alpha1),  mt.sin(theta1)*mt.sin(alpha1), 0],\
                            [mt.sin(theta1),  mt.cos(theta1)*mt.cos(alpha1), -mt.cos(theta1)*mt.sin(alpha1), 0],\
                            [0,               mt.sin(alpha1),                 mt.cos(alpha1),                prism1],\
                            [0, 0, 0, 1]])
        
        T_2_3 = np.array([  [mt.cos(alpha2), -mt.sin(theta2)*mt.cos(alpha2),  mt.sin(theta2)*mt.sin(alpha2), 0],\
                            [mt.sin(theta2),  mt.cos(theta2)*mt.cos(alpha2), -mt.cos(theta2)*mt.sin(alpha2), 0],\
                            [0,               mt.sin(alpha2),                 mt.cos(alpha2),                prism2],\
                            [0, 0, 0, 1]])
        
        T_3_4 = np.array([  [mt.cos(alpha3), -mt.sin(theta3)*mt.cos(alpha3),  mt.sin(theta3)*mt.sin(alpha3), 0],\
                            [mt.sin(theta3),  mt.cos(theta3)*mt.cos(alpha3), -mt.cos(theta3)*mt.sin(alpha3), 0],\
                            [0,               mt.sin(alpha3),                 mt.cos(alpha3),                prism3],\
                            [0, 0, 0, 1]])
        
        T_4_5 = np.array([  [mt.cos(alpha4), -mt.sin(theta4)*mt.cos(alpha4),  mt.sin(theta4)*mt.sin(alpha4), 0],\
                            [mt.sin(theta4),  mt.cos(theta4)*mt.cos(alpha4), -mt.cos(theta4)*mt.sin(alpha4), 0],\
                            [0,               mt.sin(alpha4),                 mt.cos(alpha4),                prism4],\
                            [0, 0, 0, 1]])
        
        T_5_6 = np.array([  [mt.cos(alpha5), -mt.sin(theta5)*mt.cos(alpha5),  mt.sin(theta5)*mt.sin(alpha5), 0],\
                            [mt.sin(theta5),  mt.cos(theta5)*mt.cos(alpha5), -mt.cos(theta5)*mt.sin(alpha5), 0],\
                            [0,               mt.sin(alpha5),                 mt.cos(alpha5),                prism5],\
                            [0, 0, 0, 1]])

        
        # tMatrixTrans*tMatrixRotX*tMatrixRotY
        T_0_2 = np.dot(T_1_2, T_0_1)
        T_0_3 = np.dot(T_2_3, T_0_2)
        T_0_4 = np.dot(T_3_4, T_0_3)
        T_0_5 = np.dot(T_4_5, T_0_4)
        T_0_6 = np.dot(T_5_6, T_0_5)

        listOfLists = np.transpose(T_0_6).tolist()
        print(listOfLists)

        # print(np.transpose(tMatrixRobot).tolist())
        flat_list = [x for xs in listOfLists for x in xs]
        self.tranMatrix = LMatrix4f(*flat_list)
        
        # Apply transform to assembly (position + rotations)
        self.RobotAssembly.set_mat(self.tranMatrix)


    def check_input(self, task):
        """Poll inputList for updates (simulating Tkinter shared state)."""
        try:
            anglesAndPosition = self.inputList
            angles = [anglesAndPosition[0], anglesAndPosition[1]]
            prism = anglesAndPosition[2]
            shaftPosit = [anglesAndPosition[3],
                          anglesAndPosition[4],
                          anglesAndPosition[5]]

            if ((angles != self.lastAngles) or (prism != self.lastPrismatic)):
                # print(angles)
                self.update_robot(angles, prism, shaftPosit)
                self.lastAngles = angles
                self.lastPrismatic = prism
        except Exception as e:
            print("Error:", e)



    def check_input_gantry(self, task):
        """Poll inputList for updates (simulating Tkinter shared state)."""
        try:
            self.jointSpace = self.inputList

            if (self.jointSpace != self.lastJointSpace):
                # print(angles)
                self.update_robot_gantry(self.jointSpace)
                self.lastJointSpace = self.jointSpace
        except Exception as e:
            print("Error:", e)

        return task.cont


def run_viewer(inputList):
    app = RobotViewer(inputList)
    app.run()


if __name__ == "__main__":
    # Test with dummy values
    # Axial, rotary, extension, wrist, grasper
    run_viewer([0.5, 0.3, 20, 10, 10, 10])
