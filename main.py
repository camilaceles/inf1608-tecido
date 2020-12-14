import math
import numpy as np
import pyvista as pv

def init():
  """Sets up points and faces in mesh"""
  point_index = 0
  for j in range(0, size_x):
    for i in range(0, size_y):
      points.append([j, i, 0])
      if j<size_x-1 and i<size_y-1:
        faces.append([4, point_index, point_index+1, point_index+size_y+1, point_index+size_y])
        bars.append([point_index, point_index+1])
        bars.append([point_index, point_index+size_y+1])
        bars.append([point_index, point_index+size_y])
      elif j==size_x-1 and i<size_y-1:
        bars.append([point_index, point_index+1])
      elif i==size_y-1 and j<size_x-1:
        bars.append([point_index, point_index+size_y])
      
      if j%2 == 0: # skipping odd columns for secondary bars
        if j<size_x-2 and i<size_y-2:
          secondary_bars.append([point_index, point_index+2])
          secondary_bars.append([point_index, point_index+(2*(size_y+1))])
          secondary_bars.append([point_index, point_index+(2*size_y)])
        elif j==size_x-1 and i<size_y-2:
          secondary_bars.append([point_index, point_index+2])
        elif i==size_y-1 and j<size_x-2:
          secondary_bars.append([point_index, point_index+(2*size_y)])

      point_index += 1

# Size of mesh (cloth) in points. They are all 1 unit apart.
size_x = 40
size_y = 25
straight_bar_size = 1
diagonal_bar_size = math.sqrt(2)

points = []
faces = []
bars = []
secondary_bars = []
init()

# Plotting mesh
plot = pv.Plotter()
mesh = pv.PolyData(np.array(points), np.array(faces))
plot.add_mesh(mesh, color='w', show_edges=True)
plot.show()