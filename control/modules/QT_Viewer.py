import sys
import vtk
from PyQt5 import QtWidgets, QtCore
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import math as mt

class RobotViewer(QtWidgets.QMainWindow):
    def __init__(self, inputList, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Robot Arm Viewer")

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
        self.style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetInteractorStyle(self.style)

        # Take input from tkinter
        # self.angleList = None
        # self.prismShaft = None
        # self.shaftBasePos = None 
        self.inputList = inputList

        # Build a robot link
        self.shaftCyl = None
        self.shaftLength = 30
        self.shaftRadius = 1.5
        self.robotShaft = self.make_link([self.shaftLength, self.shaftRadius])  # base link
        self.robotShaft.SetOrientation(-90, 0, 0)
        self.shaftColor = (0.50, 0.75, 0.95)
        self.robotShaft.GetProperty().SetColor(self.shaftColor)
        self.lastAngles = [None, None]

        # Axes for the link
        axesRobot = vtk.vtkAxesActor()
        axesRobot.SetTotalLength(10, 10, 10)  # scale axes to fit link

        # Group them together
        self.RobotAssembly = vtk.vtkAssembly()
        self.RobotAssembly.AddPart(self.robotShaft)
        self.RobotAssembly.AddPart(axesRobot)

        self.ren.AddActor(self.RobotAssembly)

        self.ren.ResetCamera()

        # Add 3D axes at origin that can't be moved
        self.axes = vtk.vtkAxesActor()
        self.axes.SetPosition(0, 0, 0)
        self.axes.SetOrigin(0, 0, 0)
        self.axes.SetTotalLength(5.0, 5.0, 5.0)  # adjust axis lengths
        self.axes.SetShaftTypeToCylinder()
        self.axes.SetNormalizedShaftLength(0.8, 0.8, 0.8)
        self.axes.SetAxisLabels(0)
        # self.xprop = self.axes.GetXAxisCaptionActor2D().GetCaptionTextProperty()
        # self.yprop = self.axes.GetYAxisCaptionActor2D().GetCaptionTextProperty()
        # self.zprop = self.axes.GetZAxisCaptionActor2D().GetCaptionTextProperty()

        # for prop in (self.xprop, self.yprop, self.zprop):
        #     prop.SetFontFamilyToArial()   # or ToTimes, ToCourier
        #     prop.SetFontSize(1)          # font size in pixels
        #     prop.BoldOn()
        #     prop.ItalicOff()
        #     prop.ShadowOff()

        self.ren.AddActor(self.axes)

        # rgba = [0] * 4
        # self.colors.GetColor('Carrot', rgba)
        # self.widget = vtk.vtkOrientationMarkerWidget()
        # self.widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
        # self.widget.SetOrientationMarker(self.axes)
        # self.widget.SetInteractor(self.vtkWidget)
        # self.widget.SetViewport(0.0, 0.0, 0.4, 0.4)
        # self.widget.SetEnabled(1)
        # self.widget.InteractiveOff()

        # Poll queue for joint updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.check_queue)
        self.timer.start(50)

        self.iren.Initialize()
        self.iren.Start()


    def make_link(self, dims):
        """Make a cylindrical link."""
        self.shaftCyl = vtk.vtkCylinderSource()
        self.shaftCyl.SetRadius(dims[1])
        self.shaftCyl.SetHeight(dims[0])

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.shaftCyl.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        # actor.SetPosition(0, 0, dims[0]/2)
        return actor


    def update_robot(self, angles, prismLen, shaftPosit):
        """Update robot joint positions based on angles (degrees)."""
        inclination = angles[0]*180/mt.pi - mt.pi/2
        azimuth = -angles[1]*180/mt.pi
        self.shaftCyl.SetHeight(self.shaftLength + prismLen) # Account for prismatic extension
        self.robotShaft.SetPosition(0, 0, (self.shaftLength + prismLen)/2) # Maintain local axes at shaft base

        # Shaft rotations and positionng of base at end of continuum joint
        transform = vtk.vtkTransform()
        transform.Translate(shaftPosit)
        transform.RotateX(inclination)
        transform.RotateY(azimuth)
        self.RobotAssembly.SetUserTransform(transform) # Assembly is local axes and shaft together

        self.vtkWidget.GetRenderWindow().Render()


    def check_queue(self):
        """Check for new joint angles from Tkinter process."""
        try:
            anglesAndPosition = self.inputList
            angles = [anglesAndPosition[0], anglesAndPosition[1]] #list(self.angleList) # Looks at shared memory from tkinter
            prism = anglesAndPosition[2]
            shaftPosit = [anglesAndPosition[3], anglesAndPosition[4], anglesAndPosition[5]] #list(self.shaftBasePos)
            # angles = [angles[0] + 0.1, angles[1] + 0.1]
            # self.angleList = angles
            if angles != self.lastAngles:
                self.update_robot(angles, prism, shaftPosit)
                self.last_angles = angles
        except Exception as e:
            print(e)
            pass


def run_viewer(inputList):
    app = QtWidgets.QApplication(sys.argv)
    window = RobotViewer(inputList)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    # window = VTKViewer()
    window = RobotViewer([0, 0, 20, 10 ,10, 10])
    window.show()
    sys.exit(app.exec_())