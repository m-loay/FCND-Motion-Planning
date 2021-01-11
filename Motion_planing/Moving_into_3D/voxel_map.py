import sys
from os.path import dirname, abspath
from scipy.spatial import Voronoi, voronoi_plot_2d
from mpl_toolkits.mplot3d import Axes3D
d = dirname(dirname(abspath(__file__)))
Flying_Car_Representation = d + '\\Flying_Car_Representation'
sys.path.insert(1, Flying_Car_Representation)
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from bresenham import bresenham
from ConfgurationSpace import readData

def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def closest_point_index(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    i=0
    index = 0
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        i = i + 1
        if d < dist:
            closest_point = p
            dist = d
            index = i
    return index

def create_voxmap(data, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min)) // voxel_size
    east_size = int(np.ceil(east_max - east_min)) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        # TODO: fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
        ]

        height = int(alt + d_alt) // voxel_size
        voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], 0:height] = True

    return voxmap

def main():
    plt.rcParams['figure.figsize'] = 16, 16
    data = readData()
    voxmap = create_voxmap(data, 10)
    print(voxmap.shape)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(voxmap, edgecolor='k')
    ax.set_xlim(voxmap.shape[0], 0)
    ax.set_ylim(0, voxmap.shape[1])
    # add a bit to z-axis height for visualization
    ax.set_zlim(0, voxmap.shape[2]+20)

    plt.xlabel('North')
    plt.ylabel('East')

    plt.show()

if __name__ == "__main__":
    main()