'''
In this notebook you'll create a configuration space given a map of the world and setting a particular altitude 
for your drone. 
You'll read in a .csv file containing obstacle data which consists of six columns  𝑥 ,  𝑦 ,  𝑧  and  𝛿𝑥 ,  𝛿𝑦 ,  𝛿𝑧 .

You can look at the .csv file here. The first line gives the map center coordinates and the file is arranged such that:
𝑥  -> NORTH
𝑦  -> EAST
𝑧  -> ALTITUDE (positive up, note the difference with NED coords)
Each  (𝑥,𝑦,𝑧)  coordinate is the center of an obstacle.  𝛿𝑥 ,  𝛿𝑦 ,  𝛿𝑧  
are the half widths of the obstacles,
meaning for example that an obstacle with  (𝑥=37,𝑦=12,𝑧=8)  and  (𝛿𝑥=5,𝛿𝑦=5,𝛿𝑧=8)  
is a 10 x 10 m obstacle that is 16 m high and is centered at the point  (𝑥,𝑦)=(37,12)  at a height of 8 m.

Given a map like this, the free space in the  (𝑥,𝑦)  plane is a function of altitude, 
and you can plan a path around an obstacle, or simply fly over it! You'll extend each obstacle 
by a safety margin to create the equivalent of a 3 dimensional configuration space.

Your task is to extract a 2D grid map at 1 metre resolution of your configuration space for a 
particular altitude, where each value is assigned either a 0 or 1 representing 
feasible or infeasible (obstacle) spaces respectively.

'''

import numpy as np 
import matplotlib.pyplot as plt

'''
The given function will take the data from the file describing the obstacles city and will return a 2D grid 
representation showing open and closed spaces. 
'''
def readData(filename= 'colliders.csv'):
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
    return data
    
def create_grid(data, drone_altitude =5, safety_distance=3):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid

def main():
    plt.rcParams["figure.figsize"] = [12, 12]

    # Static drone altitude (metres)
    drone_altitude = 5

    # Minimum distance required to stay away from an obstacle (metres)
    # Think of this as padding around the obstacles.
    safe_distance = 3

    data = readData()
    grid = create_grid(data,drone_altitude, safe_distance)
    # equivalent to
    # plt.imshow(np.flip(grid, 0))
    # NOTE: we're placing the origin in the lower lefthand corner here
    # so that north is up, if you didn't do this north would be positive down
    plt.imshow(grid, origin='lower') 

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

if __name__ == "__main__":
    main()