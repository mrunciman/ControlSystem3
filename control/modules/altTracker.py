


import pyvista as pv
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

# Load mesh
original_mesh  = pv.read("C:\\Users\\msrun\\Documents\\Inflatable Robot Control\\ControlSystem3\\control\\modules\\BowdenAdapter.STL")  # Replace with actual path

# Create a plotter in interactive mode
plotter = pv.Plotter()
actor = plotter.add_mesh(original_mesh.copy(), color="steelblue", show_edges=False)

# Add coordinate axes for context
plotter.add_axes()
plotter.add_mesh(pv.Sphere(radius=0.5, center=(0, 0, 0)), color='red')
plotter.show(auto_close=False, interactive_update=True)  # Keep window open

# Loop to update pose
for i in range(180):
    # Calculate rotation matrix (rotate around Z)
    angle = i * 5  # degrees
    rotation = R.from_euler('z', angle, degrees=True).as_matrix()
    
    # Build 4x4 transformation matrix
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation
    transform_matrix[:3, 3] = [-1, 0, 0]  # No translation

    # Transform mesh
    mesh = original_mesh.copy()
    mesh.transform(transform_matrix)

    # Clear previous mesh and show new one
    plotter.clear_actors()  # <-- important
    plotter.add_mesh(mesh, color="steelblue", show_edges=True)
    plotter.add_mesh(pv.Sphere(radius=0.5, center=(0, 0, 0)), color='red')
    plotter.render()

    time.sleep(0.05)  # Adjust for animation speed
plotter.close()

# Close window manually or with plotter.close()