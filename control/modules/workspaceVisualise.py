# import numpy as np
# import matplotlib.pyplot as plt
# import math as mt

# # 1. Define your robot's joint limits (adjust these to your actual hardware)
# limits = {
#     'prism0': (0.0, 0.5),      # Axial extension range (meters)
#     'theta1': (0, 1),     # Rotary angle range (degrees)
#     'theta3': (0, 10),       # Wrist angle range (degrees)
#     'prism6': (0.0, 0.3)       # Tool extension range (meters)
# }

# num_samples = 20000  # Number of points to generate

# # 2. Randomly sample the joint space uniformly
# p0 = np.random.uniform(limits['prism0'][0], limits['prism0'][1], num_samples)
# t1 = np.radians(np.random.uniform(limits['theta1'][0], limits['theta1'][1], num_samples))
# # Don't forget your original offset: theta3 = jointSpace[3] + pi/2
# t3 = np.radians(np.random.uniform(limits['theta3'][0], limits['theta3'][1], num_samples)) + np.pi/2
# p6 = np.random.uniform(limits['prism6'][0], limits['prism6'][1], num_samples)

# # 3. Vectorized Forward Kinematics (Extracted from T_0_7)
# x = p6*np.sin(t1) * np.cos(t3)
# y = -p6*np.cos(t1) * np.cos(t3)
# z = p0 - p6 * np.sin(t3)

# # 4. Plot the workspace in 3D
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# # Color the points by their Z-height to add depth perception
# sc = ax.scatter(x, y, z, c=z, cmap='viridis', s=1, alpha=0.6)

# ax.set_xlabel('X Position')
# ax.set_xlim(-0.3, 0)
# ax.set_ylabel('Y Position')
# ax.set_zlabel('Z Position')
# ax.set_title('Robot Workspace Visualization')
# # fig.colorbar(sc, label='Z Depth')
# ax.set_box_aspect((1, 1, 1))

# plt.show()




import math as mt
import numpy as np
from panda3d.core import (
    GeomVertexData, GeomVertexFormat, GeomVertexWriter, 
    GeomPoints, Geom, GeomNode, TransparencyAttrib
)
from direct.showbase.ShowBase import ShowBase

# ==========================================
# 1. ROBOT CONFIGURATION & LIMITS
# ==========================================
LIMITS = {
    'p0': (0.0, 0.5),       # Prismatic base lift
    # 't1': (-mt.pi, mt.pi),  # Revolute base rotation
    't1': (0, mt.pi),  # Revolute base rotation
    't3': (0, mt.pi/2), # Revolute shoulder tilt
    'p6': (0.1, 0.3)        # Prismatic arm extension
}

def is_point_reachable(x, y, z):
    """
    Checks if a 3D coordinate is within the robot's physical kinematics limits.
    Returns True if reachable, False otherwise.
    """
    # Step A: Check Theta 1
    # From FK: x = p6*sin(t1)*cos(t3), y = -p6*cos(t1)*cos(t3) -> tan(t1) = x / -y
    theta1 = mt.atan2(x, -y)
    if not (LIMITS['t1'][0] <= theta1 <= LIMITS['t1'][1]):
        return False
        
    # Step B: Handle Redundancy (Scan possible p0 configurations)
    # R_xy is the horizontal projection distance from the central column
    R_xy = mt.sqrt(x**2 + y**2)
    
    # We test a discrete set of possible p0 extensions to see if any work
    p0_samples = np.linspace(LIMITS['p0'][0], LIMITS['p0'][1], num=20)
    
    for p0 in p0_samples:
        # Distance needed along Z axis for this specific p0
        z_needed = p0 - z
        
        # Calculate required p6 extension using Pythagorean theorem
        p6_needed = mt.sqrt(R_xy**2 + z_needed**2)
        
        # Check if the required arm length fits the physical link limits
        if LIMITS['p6'][0] <= p6_needed <= LIMITS['p6'][1]:
            # Calculate required Theta 3 angle for this configuration
            # In your equations, offset was applied: t3_adjusted = t3 + pi/2
            theta3 = mt.atan2(z_needed, R_xy)
            
            if LIMITS['t3'][0] <= theta3 <= LIMITS['t3'][1]:
                return True # A valid configuration exists!
                
    return False # Exhausted all options; point is completely unreachable


# ==========================================
# 2. PANDA3D VISUALIZATION ENGINE
# ==========================================
class WorkspaceScanner(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        # Set up a clean background & camera orientation
        self.setBackgroundColor(0.1, 0.1, 0.1)
        self.cam.setPos(0, -3, 2)
        self.cam.lookAt(0, 0, 0.25)
        
        # Generate the filtered point cloud
        self.visualize_sampled_space()

    def visualize_sampled_space(self):
        # Setup modern vertex formats for rendering Point Primitives
        v_format = GeomVertexFormat.get_v3c4() 
        v_data = GeomVertexData('scan_data', v_format, Geom.UHStatic)
        
        vertex_writer = GeomVertexWriter(v_data, 'vertex')
        color_writer = GeomVertexWriter(v_data, 'color')
        prim = GeomPoints(Geom.UHStatic)
        
        # Create a 3D bounding box grid surrounding the robot link bounds
        x_space = np.linspace(-0.4, 0.4, 30)
        y_space = np.linspace(-0.4, 0.4, 30)
        z_space = np.linspace(-0.3, 0.7, 30)
        
        point_idx = 0
        
        # Uniformly sample space
        for x in x_space:
            for y in y_space:
                for z in z_space:
                    vertex_writer.addData3(x, y, z)
                    
                    # Run our IK check
                    if is_point_reachable(x, y, z):
                        # Reachable -> Translucent Green Node
                        color_writer.addData4(0.1, 0.9, 0.1, 0.5)
                    else:
                        # Unreachable / Dead-zone -> Very faint Translucent Red Node
                        color_writer.addData4(0.9, 0.1, 0.1, 0.02)
                    
                    prim.addVertex(point_idx)
                    point_idx += 1
                    
        prim.closePrimitive()
        geom = Geom(v_data)
        geom.addPrimitive(prim)
        
        geom_node = GeomNode('workspace_scan')
        geom_node.addGeom(geom)
        
        # Attach to rendering pipeline
        scan_np = self.render.attachNewNode(geom_node)
        scan_np.setRenderModeThickness(4.0)
        scan_np.setTransparency(TransparencyAttrib.MAlpha)

if __name__ == '__main__':
    app = WorkspaceScanner()
    app.run()
