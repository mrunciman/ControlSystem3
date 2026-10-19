import math

def make_cylinder_egg(filename, radius, height, slices, u_repeat):
    with open(filename, "w") as f:
        f.write("<CoordinateSystem> { Z-up }\n\n")
        f.write("<Group> Cylinder {\n")
        f.write("  <VertexPool> vpool {\n")

        # bottom circle vertices (z = 0)
        for i in range(slices):
            angle = 2 * math.pi * i / slices
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            # outward normal on the side
            nx, ny, nz = math.cos(angle), math.sin(angle), 0
            u = (i / slices) * u_repeat
            v = 0.0
            f.write(f"    <Vertex> {i} {{ {x:.6f} {y:.6f} 0.0 <Normal> {{ {nx:.6f} {ny:.6f} {nz:.6f} }} <UV> {{ {u:.6f} {v:.6f} }}  }}\n")
            # f.write(f"    <Vertex> {i} {{ {x:.6f} {y:.6f} {0.0:.6f} }}\n")

        # top circle vertices (z = h)
        for i in range(slices):
            angle = 2 * math.pi * i / slices
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            nx, ny, nz = math.cos(angle), math.sin(angle), 0
            u = (i / slices) * u_repeat
            v = 1.0
            f.write(f"    <Vertex> {i+slices} {{ {x:.6f} {y:.6f} {height:.6f} <Normal> {{ {nx:.6f} {ny:.6f} {nz:.6f} }} <UV> {{ {u:.6f} {v:.6f} }}  }}\n")
            # f.write(f"    <Vertex> {i+slices} {{ {x:.6f} {y:.6f} {height:.6f} }}\n")

        f.write("  }\n\n")

        # bottom face (fan)
        f.write("  <Polygon> { <Normal> { 0 0 -1 } <VertexRef> { ")
        for i in range(slices):
            f.write(f"{slices-1-i} ")  # reverse order for correct normal
        f.write("<Ref> { vpool } } }\n")

        # top face (fan)
        f.write("  <Polygon> { <Normal> { 0 0 1 } <VertexRef> { ")
        for i in range(slices):
            f.write(f"{i+slices} ")
        f.write("<Ref> { vpool } } }\n")

        # side faces (quads)
        for i in range(slices):
            i_bottom = i
            i_top = i + slices
            j_bottom = (i + 1) % slices
            j_top = j_bottom + slices
            f.write("  <Polygon> { <VertexRef> { ")
            f.write(f"{i_bottom} {j_bottom} {j_top} {i_top} ")
            f.write("<Ref> { vpool } } }\n")


        f.write("}\n")


if __name__ == "__main__":
    make_cylinder_egg("./control/assets/cylinder.egg", radius=1.5, height=30.0, slices=32, u_repeat=1)
    print("Generated cylinder.egg with 32 sides")

