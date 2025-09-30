from direct.showbase.ShowBase import ShowBase
# import panda3d
from panda3d.core import LineSegs, NodePath
from panda3d.core import WindowProperties
# from panda3d.core import Cylinder
import math as mt


class RobotViewer(ShowBase):
    def __init__(self, inputList):
        super().__init__()
        props = WindowProperties()
        props.setTitle("Robot Arm Viewer (Panda3D)")
        self.win.requestProperties(props)
        # self.setWindowTitle("Robot Arm Viewer (Panda3D)")

        # Camera setup
        self.setBackgroundColor(0.1, 0.1, 0.1, 1)
        # self.camera.setPos(50, -80, 50)
        self.camera.lookAt(0, 0, 0)

        # Store input reference (list from Tkinter/shared memory)
        self.inputList = inputList
        self.lastAngles = [None, None]

        # Shaft parameters
        self.shaftLength = 30
        self.shaftRadius = 1.5

        # Make shaft and add local axes
        self.robotShaft = self.make_cylinder(self.shaftRadius, self.shaftLength)
        self.robotShaft.setColor(0.5, 0.75, 0.95, 1)
        self.robotShaft.setHpr(-90, 0, 0)

        axes_local = self.make_axes(length=10)
        axes_local.reparentTo(self.robotShaft)

        self.RobotAssembly = NodePath("RobotAssembly")
        self.robotShaft.reparentTo(self.RobotAssembly)

        self.RobotAssembly.reparentTo(self.render)

        # Global axes
        axes_global = self.make_axes(length=5)
        axes_global.reparentTo(self.render)

        # Update task
        self.taskMgr.add(self.check_input, "CheckInputTask")

    def make_cylinder(self, radius, height, slices=32):
        """Use Panda3D's built-in geometry for a cylinder."""
        cyl = self.loader.loadModel("models/teapot")
        cyl.reparent_to(self.render)
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
        inclination = angles[0] * 180 / mt.pi - 90
        azimuth = -angles[1] * 180 / mt.pi

        # Scale cylinder along Z for prismatic extension
        scale_z = (self.shaftLength + prismLen) / self.shaftLength
        self.robotShaft.setScale(1, 1, scale_z)

        # Keep base at origin by moving it half height
        self.robotShaft.setZ((self.shaftLength + prismLen) / 2.0)

        # Apply transform to assembly (position + rotations)
        self.RobotAssembly.setPos(*shaftPosit)
        self.RobotAssembly.setHpr(inclination, azimuth, 0)

    def check_input(self, task):
        """Poll inputList for updates (simulating Tkinter shared state)."""
        try:
            anglesAndPosition = self.inputList
            angles = [anglesAndPosition[0], anglesAndPosition[1]]
            prism = anglesAndPosition[2]
            shaftPosit = [anglesAndPosition[3],
                          anglesAndPosition[4],
                          anglesAndPosition[5]]

            if angles != self.lastAngles:
                print(angles)
                self.update_robot(angles, prism, shaftPosit)
                self.lastAngles = angles
        except Exception as e:
            print("Error:", e)

        return task.cont


def run_viewer(inputList):
    app = RobotViewer(inputList)
    app.run()


if __name__ == "__main__":
    # Test with dummy values
    run_viewer([0.5, 0.3, 20, 10, 10, 10])
