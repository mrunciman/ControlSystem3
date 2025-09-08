import sys
import vtk
from PyQt5 import QtWidgets, QtCore
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import multiprocessing

class RobotViewer(QtWidgets.QMainWindow):
    def __init__(self, queue, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Robot Arm Viewer")
        self.queue = queue

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
        self.joint1 = self.make_link([1, 0.2, 0.2])  # base link
        self.joint2 = self.make_link([0.8, 0.2, 0.2])  # second link

        self.ren.AddActor(self.joint1)
        self.ren.AddActor(self.joint2)

        self.ren.ResetCamera()

        # Poll queue for joint updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.check_queue)
        self.timer.start(1)

        self.iren.Initialize()
        self.iren.Start()

    def make_link(self, dims):
        """Make a rectangular link (cube scaled)."""
        cube = vtk.vtkCubeSource()
        cube.SetXLength(dims[0])
        cube.SetYLength(dims[1])
        cube.SetZLength(dims[2])
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return actor

    def update_robot(self, angles):
        """Update robot joint positions based on angles (degrees)."""
        theta1, theta2 = angles

        # Joint 1 rotation around Z
        self.joint1.SetOrientation(0, 0, theta1)
        self.joint1.SetPosition(0, 0, 0)

        # Joint 2 attached at end of joint1
        self.joint2.SetOrientation(0, 0, theta1 + theta2)
        self.joint2.SetPosition(
            0.5 * 1.0,  # half of link1 length in X
            0, 0
        )

        self.vtkWidget.GetRenderWindow().Render()

    def check_queue(self):
        """Check for new joint angles from Tkinter process."""
        try:
            while not self.queue.empty():
                angles = self.queue.get_nowait()
                self.update_robot(angles)
        except Exception:
            pass

# class VTKViewer(QtWidgets.QMainWindow):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("VTK 3D Viewer")

#         # Central widget
#         self.frame = QtWidgets.QFrame()
#         self.layout = QtWidgets.QVBoxLayout()
#         self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
#         self.layout.addWidget(self.vtkWidget)
#         self.frame.setLayout(self.layout)
#         self.setCentralWidget(self.frame)

#         # Setup VTK scene
#         self.ren = vtk.vtkRenderer()
#         self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
#         self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

#         # Example: cube
#         cube = vtk.vtkCubeSource()
#         mapper = vtk.vtkPolyDataMapper()
#         mapper.SetInputConnection(cube.GetOutputPort())
#         actor = vtk.vtkActor()
#         actor.SetMapper(mapper)

#         self.ren.AddActor(actor)
#         self.ren.ResetCamera()

#         # Animation timer
#         self.timer = QtCore.QTimer()
#         self.timer.timeout.connect(lambda: self.rotate(actor))
#         self.timer.start(50)

#         self.iren.Initialize()
#         self.iren.Start()

#     def rotate(self, actor):
#         actor.RotateY(0.5)
#         self.vtkWidget.GetRenderWindow().Render()

def run_viewer(queue):
    app = QtWidgets.QApplication(sys.argv)
    window = RobotViewer(queue)
    window.show()
    sys.exit(app.exec_())

# if __name__ == "__main__":
#     app = QtWidgets.QApplication(sys.argv)
#     # window = VTKViewer()
#     window = RobotViewer(queue)
#     window.show()
#     sys.exit(app.exec_())