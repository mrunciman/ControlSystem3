from direct.showbase.ShowBase import ShowBase
from panda3d.core import LineSegs, NodePath, WindowProperties, LMatrix4f, PointLight, Texture, Material
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
        self.axes_local.reparentTo(self.RobotAssembly)
        self.RobotAssembly.reparentTo(self.render)

        # Global axes
        self.axes_global = self.make_axes(length=10)
        self.axes_global.reparentTo(self.render)

        self.tranMatrix = self.RobotAssembly.get_mat(self.axes_global)

        # Camera setup
        self.setBackgroundColor(0.8, 0.8, 0.85, 1)
        self.cam.setPos(0, 0, 300)
        self.cam.lookAt(0,0,0)

        # Add a light to the scene
        plight = PointLight('plight')
        # plight.attenuation = (1, 0, 1)
        plnp = self.render.attachNewNode(plight)
        plnp.setPos(-100, 0, 100)
        self.render.setLight(plnp)

        # Update task
        self.taskMgr.add(self.check_input, "CheckInputTask")

    def make_cylinder(self, radius, height):
        """Use Panda3D's built-in geometry for a cylinder."""
        cyl = self.loader.loadModel("models/cylinder")
        tex = self.loader.loadTexture("models/LOGO.png")

        # tex.setWrapU(Texture.WM_border_color)
        # tex.setWrapV(Texture.WM_border_color)
        # tex.setBorderColor((0.0, 0.0, 0*160/255, 1))
        # cyl.setTexture(tex, 1)

        # ts = TextureStage.getDefault()
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

        return task.cont


def run_viewer(inputList):
    app = RobotViewer(inputList)
    app.run()


if __name__ == "__main__":
    # Test with dummy values
    run_viewer([0.5, 0.3, 20, 10, 10, 10])
