import math
from time import sleep
import numpy as np
import pyvista as pv

#-------------------------------------------------------------------
# Global variables
#-------------------------------------------------------------------
# Size of mesh (cloth) in points. They are all 1 unit apart.
size_x = 40
size_y = 25
straight_bar_size = 1
diagonal_bar_size = math.sqrt(2)

mass = 0.007 # mass of each point
wind_force = [-2, 2, -5]
gravity = [0, -9.8, 0]

points = [] # [x, y, z]
last_points = [] # stores (i-1) data for points
faces = [] # [num_vertices, vertices_indexes...]
bars = [] # [vertice_index, vertice_index, size]
secondary_bars = [] # [vertice_index, vertice_index, size]

camera_pos = [
  (10, 25, 120), # camera location
  (20, 0, 0),    # focus point
  (0, 0, 0),     # viewup vector
]

#-------------------------------------------------------------------
# Auxiliar methods
#-------------------------------------------------------------------
def is_mobile(index):
  """Returns wether the point with given index is mobile or not"""
  return index >= size_y

#-------------------------------------------------------------------
# Methods
#-------------------------------------------------------------------
def impose_constraint():
  """Imposes constraint on bars"""
  for k in range(0, 20): # TODO how many times?
    for i,bar in enumerate(bars+secondary_bars):
      a = points[bar[0]]
      b = points[bar[1]]
      d = np.subtract(b, a)
      dist = np.linalg.norm(d)
      u = np.multiply(d, 1/dist)
      dif = 0.9*(dist - bar[2])/2
      if is_mobile(bar[0]):
        points[bar[0]] = np.add(a, np.multiply(u, dif))
      if is_mobile(bar[1]):
        points[bar[1]] = np.add(b, np.multiply(u, -dif))

def animate(h):
  """Animates mesh"""
  global points, last_points
  points_i = points.copy() # storing points in current state to later save them in last_points
  damp_ratio = 0.02

  # First, we move each point independently
  for i,point in enumerate(points):
    if is_mobile(i):
      point[0] = point[0] + ((1 - damp_ratio)*(point[0] - last_points[i][0])) + (h*h/mass)*wind_force[0]
      point[1] = point[1] + ((1 - damp_ratio)*(point[1] - last_points[i][1])) + (h*h/mass)*gravity[1]
      point[2] = point[2] + ((1 - damp_ratio)*(point[2] - last_points[i][2])) + (h*h/mass)*wind_force[2]

  # Then, we impose constraint bars restrictions
  impose_constraint()
  last_points = points_i

def init():
  """Sets up points and faces in mesh"""
  point_index = 0
  for j in range(0, size_x):
    for i in range(0, size_y):
      points.append([j, i, 0])
      if j<size_x-1 and i<size_y-1:
        faces.append([4, point_index, point_index+1, point_index+size_y+1, point_index+size_y])
        bars.append([point_index, point_index+1, straight_bar_size])
        bars.append([point_index, point_index+size_y+1, diagonal_bar_size])
        bars.append([point_index, point_index+size_y, straight_bar_size])
      elif j==size_x-1 and i<size_y-1:
        bars.append([point_index, point_index+1, straight_bar_size])
      elif i==size_y-1 and j<size_x-1:
        bars.append([point_index, point_index+size_y, straight_bar_size])
      
      if j%2 == 0: # skipping odd columns for secondary bars
        if j<size_x-2 and i<size_y-2:
          secondary_bars.append([point_index, point_index+2, 2*straight_bar_size])
          secondary_bars.append([point_index, point_index+(2*(size_y+1)), 2*diagonal_bar_size])
          secondary_bars.append([point_index, point_index+(2*size_y), 2*straight_bar_size])
        elif j==size_x-1 and i<size_y-2:
          secondary_bars.append([point_index, point_index+2, 2*straight_bar_size])
        elif i==size_y-1 and j<size_x-2:
          secondary_bars.append([point_index, point_index+(2*size_y), 2*straight_bar_size])

      point_index += 1

  global last_points
  last_points = points.copy()

#-------------------------------------------------------------------
# Execution
#-------------------------------------------------------------------
init()
print("To star the animation, click 'q'")

# Plotting mesh
plotter = pv.Plotter()
mesh = pv.PolyData(np.array(points), np.array(faces))
plotter.add_mesh(mesh, color='w', show_edges=True, interpolate_before_map=True)
plotter.add_axes()
plotter.enable_eye_dome_lighting()
plotter.camera_position = camera_pos
plotter.show(interactive=True, auto_close=False, window_size=[800, 600])


plotter.open_gif("animation.gif")

for i in range(0, 70):
  animate(0.033)
  plotter.update_coordinates(np.array(points), mesh=mesh)
  plotter.write_frame()

plotter.close()