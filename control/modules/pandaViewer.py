from direct.showbase.ShowBase import ShowBase
from panda3d.core import LineSegs, NodePath, WindowProperties, LMatrix4f, PointLight, Texture, Material, TextureStage, DirectionalLight, AmbientLight
from direct.gui.OnscreenImage import OnscreenImage
import math as mt
import numpy as np


class RobotViewer(ShowBase):
    def __init__(self, inputList):
        ShowBase.__init__(self)
        props = WindowProperties()
        props.setTitle("Motion Visualisation")
        self.win.requestProperties(props)

        # Load the background image into Panda3D's background 2D layer (render2dp)
        # Note: If you want the image to stretch and completely fill the window, use render2dp.
        # If you want to preserve the aspect ratio, use base.aspect2dp instead.
        if __name__ == "__main__":
            self.background = OnscreenImage(image="ColonMockup.png", parent=base.render2dp)
        else:
            self.background = OnscreenImage(image="control/assets/ColonMockup.png", parent=base.render2dp)
        # Change the sort order of the background layer so it draws BEFORE the 3D scene
        # Default 3D scene sort is 0, so setting this to -20 ensures it draws first.
        base.cam2dp.node().getDisplayRegion(0).setSort(-20)
        # CRITICAL: Stop the main 3D camera from clearing the color buffer.
        # If left True, the 3D camera will paint a solid gray/black background 
        # color right over your image when it starts rendering the robot
        base.cam.node().getDisplayRegion(0).setClearColorActive(False)

        # Store input reference (list from Tkinter/shared memory)
        self.inputList = inputList
        # self.jointSpace = None
        self.lastJointSpace = [None, None, None, None, None, None]
        self.lastAngles = [None, None]
        self.lastPrismatic = None

        # Shaft parameters
        self.cylModelLength = 30

        # Make tip of robot
        self.RobotAssembly = NodePath("RobotAssembly")
        self.axes_local = self.make_axes(length=10)
        self.axes_local.setLightOff(1)
        self.axes_local.reparentTo(self.RobotAssembly)
        self.RobotAssembly.reparentTo(self.render)

        # Make wrist of robot
        self.RobotWrist = NodePath("RobotWrist")
        self.toolExtCyl = self.make_cylinder()
        self.toolExtCyl.reparentTo(self.RobotWrist)
        self.axes_local = self.make_axes(length=10)
        self.axes_local.setLightOff(1)
        self.axes_local.reparentTo(self.RobotWrist)
        self.RobotWrist.reparentTo(self.render)

        # Global axes
        self.axes_global = self.make_axes(length=10)
        self.axes_global.setLightOff(1)
        self.axialCyl = self.make_cylinder()
        self.axialCyl.reparentTo(self.render)
        self.workingCyl = self.make_cylinder()
        self.workingCyl.reparentTo(self.render)
        self.workingCyl.setScale(1, 1, -20)
        self.axes_global.reparentTo(self.render)

        self.tranMatrix = self.RobotAssembly.get_mat(self.axes_global)

        # Camera setup
        self.setBackgroundColor(0.0, 0.0, 0.0, 1)
        self.cam.setPos(10, 50, -250)
        # self.cam.lookAt(0,0,0)
        # The camera looks down its Y axis
        # Heading, pitch, roll control the local rotations around Z, X, and Y axes.
        # Starting from same as global axes, rotate camera axes about the X axis so the
        # camera's Y axis is aligned with Global Z axis.
        self.cam.setHpr(self.axes_global, 0, 90, 0)


        # Add a light to the scene
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

    def make_cylinder(self):
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



    def update_robot_gantry(self, jointSpace):
        """Update robot joint positions based on angles and prismatic extension."""
        # DH algorithm applies theta then alpha
        alpha0 = 0
        dist0 = 0
        theta0 = 0
        prism0 = jointSpace[0] # Axial extension        

        alpha1 = 0
        dist1 = 0
        theta1 = mt.radians(jointSpace[1]) + mt.pi # Rotary angle
        prism1 = 0 

        alpha2 = -mt.pi/2
        dist2 = 0
        theta2 = -mt.pi/2   # Intermediate 1
        prism2 = 0

        alpha3 = 0
        dist3 = 0
        theta3 = mt.radians(jointSpace[3]) # Wrist angle
        prism3 = 0

        alpha4 = mt.pi/2 # Intermediate 2
        dist4 = 0
        theta4 = 0
        prism4 = 0

        alpha5 = 0
        dist5 = 0
        theta5 = mt.pi/2 # Intermediate 3
        prism5 = 0

        alpha6 = 0
        dist6 = 0
        theta6 = 0
        prism6 = jointSpace[2] # Tool extension

        grasp = jointSpace[4]  # Grapser position

        # # --- Compute all intermediate steps natively on the GPU/C++ layer ---
        # # Pass any custom, changing variables right into the function arguments
        T_0_1 = self.make_dh_matrix(theta0, alpha0, dist0, prism0) # axial motion
        T_1_2 = self.make_dh_matrix(theta1, alpha1, dist1, prism1) # rotary motion
        T_2_3 = self.make_dh_matrix(theta2, alpha2, dist2, prism2) # intermediate
        T_3_4 = self.make_dh_matrix(theta3, alpha3, dist3, prism3) # wrist angle
        T_4_5 = self.make_dh_matrix(theta4, alpha4, dist4, prism4) # intermediate 2
        T_5_6 = self.make_dh_matrix(theta5, alpha5, dist5, prism5) # intermediate 3
        T_6_7 = self.make_dh_matrix(theta6, alpha6, dist6, prism6) # tool extension

        # # Pure C++ matrix multiplications (Incredibly fast)
        T_0_2 = T_1_2 * T_0_1 # axial and rotary motions
        T_0_3 = T_2_3 * T_0_2 # intermediate
        T_0_4 = T_3_4 * T_0_3 # apply wrist angle
        T_0_5 = T_4_5 * T_0_4 # intermed 2
        T_0_6 = T_5_6 * T_0_5 # intermed 3
        T_0_7 = T_6_7 * T_0_6 # tool extension

       
        # Apply transform to assembly (position + rotations)
        self.RobotWrist.set_mat(T_0_6)
        
        # Apply transform to assembly (position + rotations)
        self.RobotAssembly.set_mat(T_0_7)

        # Scale cylinder along Z up to the wrist
        scale_z_axial =  prism0/self.cylModelLength
        self.axialCyl.setScale(1, 1, scale_z_axial)
        # self.upToWrist.setPos(self.upToWrist, 0, 0, -prism0)

        # Scale cylinder along Z for the tool extension
        scale_z_tool = prism6/self.cylModelLength
        self.toolExtCyl.setScale(1, 1, scale_z_tool)
        # self.robotShaft.setPos(self.robotShaft, 0, 0, -prism6)




    def check_input_gantry(self, task):
        """Poll inputList for updates (simulating Tkinter shared state)."""
        try:
            localJointSpace = self.inputList
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
    run_viewer([10, -10, 20, -20, 0, 0])
