import sys
import vtk
from PyQt5 import QtWidgets, QtCore
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import math as mt

class RobotViewer(QtWidgets.QMainWindow):
    def __init__(self, angleInput, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Robot Arm Viewer")
        # self.queue = queue

        # Central widget
        self.frame = QtWidgets.QFrame()
        self.layout = QtWidgets.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        self.layout.addWidget(self.vtkWidget)
        self.frame.setLayout(self.layout)
        self.setCentralWidget(self.frame)

        # VTK setup
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        # Build a simple 2-joint robot arm
        self.angleList = angleInput
        self.robotShaft = self.make_link([30, 1.5])  # base link
        self.robotShaft.SetOrientation(-90, 0, 0)
        self.lastAngles = [0, 0]

        self.ren.AddActor(self.robotShaft)

        self.ren.ResetCamera()

        self.axes = vtk.vtkAxesActor()

        colors = vtk.vtkNamedColors()
        self.ren.SetBackground(colors.GetColor3d('SlateGray'))
        rgba = [0] * 4
        colors.GetColor('Carrot', rgba)
        self.widget = vtk.vtkOrientationMarkerWidget()
        self.widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
        self.widget.SetOrientationMarker(self.axes)
        self.widget.SetInteractor(self.vtkWidget)
        self.widget.SetViewport(0.0, 0.0, 0.4, 0.4)
        self.widget.SetEnabled(1)
        self.widget.InteractiveOn()



        # Poll queue for joint updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.check_queue)
        self.timer.start(50)

        self.iren.Initialize()
        self.iren.Start()

    def make_link(self, dims):
        """Make a cylindrical link."""
        cyl = vtk.vtkCylinderSource()
        cyl.SetRadius(dims[1])
        cyl.SetHeight(dims[0])

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cyl.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return actor

    def update_robot(self, angles):
        """Update robot joint positions based on angles (degrees)."""
        theta1 = angles[0]*180/mt.pi - 90
        theta2 = angles[1]*180/mt.pi

        # Joint 1 rotation around Z
        self.robotShaft.SetOrientation(theta1, 0, theta2)
        self.robotShaft.SetPosition(0, 0, 0)

        self.vtkWidget.GetRenderWindow().Render()

    def check_queue(self):
        """Check for new joint angles from Tkinter process."""
        try:
            angles = list(self.angleList)
            if angles != self.lastAngles:
                self.update_robot(angles)
                self.last_angles = angles
        except Exception:
            pass


def run_viewer(shared_angles):
    app = QtWidgets.QApplication(sys.argv)
    window = RobotViewer(shared_angles)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    # window = VTKViewer()
    window = RobotViewer([0, 0])
    window.show()
    sys.exit(app.exec_())