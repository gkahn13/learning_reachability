import random
import yaml
import argparse
import os

import numpy as np
import scipy.io

import matplotlib
matplotlib.use('wxagg')
import matplotlib.pyplot as plt
           
def is_valid_position(grid_size, pos):
    valid = True
    valid = valid and pos[0] >= 0
    valid = valid and pos[0] < grid_size[0]
    valid = valid and pos[1] >= 0
    valid = valid and pos[1] < grid_size[1]
    return valid
    
def obstacle_min_dist(grid, pos, not_included):
    grid = np.copy(grid)
    not_included = np.array(list(not_included))
    
    if len(not_included) > 0:
        grid[not_included[:,0], not_included[:,1]] = 0.
    obstacles = np.vstack(np.nonzero(grid)).T
    dist = np.min([np.inf] + [np.linalg.norm(pos - o, 1) for o in obstacles])
    return dist
    
def generate_grid(size, density, object_density_range, object_separation, goal_center, goal_extents):
    """
    :return obstacle grid, target grid
    """
    neighbors = np.array([[1,1], [1,0], [1,-1], [0,-1], [-1,-1], [-1,0], [-1,1], [0,1]])

    obst_grid = np.zeros(size) # free region are 0
    while abs(obst_grid).sum() / float(np.prod(obst_grid.shape)) < density:
        object_density = np.random.uniform(*object_density_range)
        center = (np.random.randint(size[0]), np.random.randint(size[1]))

        added = set()
        fringe = [center]
        while len(added) / float(np.prod(obst_grid.shape)) < object_density and len(fringe) > 0:
            random.shuffle(fringe)
            f = fringe.pop()
            assert(f not in added)
            added.add(f)
            f_children = [tuple(n + f) for n in neighbors if is_valid_position(size, n + f) and \
                                                             tuple(n + f) not in added and \
                                                             tuple(n + f) not in fringe and \
                                                             obstacle_min_dist(obst_grid, n + f, added) >= object_separation*max(size)]
            fringe += f_children
            
        if len(added) / float(np.prod(obst_grid.shape)) >= object_density_range[0]:
            added = np.array(list(added))
            obst_grid[added[:,0], added[:,1]] = -1 # obstacles marked with -1
        
    goal_x = np.arange(goal_center[0]-1 - (goal_extents[0]-1)//2, goal_center[0] + (goal_extents[0]-1)//2)
    goal_y = np.arange(goal_center[1]-1 - (goal_extents[1]-1)//2, goal_center[1] + (goal_extents[1]-1)//2)
    goal_grid = np.meshgrid(goal_x, goal_y)
    obst_grid[goal_grid[0].ravel(), goal_grid[1].ravel()] = 0 # goal region is clear

    targ_grid = np.zeros(size)
    targ_grid[goal_grid[0].ravel(), goal_grid[1].ravel()] = -1 # goal region marked with -1
    
    return obst_grid, targ_grid

def save_grid(obst_grid, targ_grid, file_name):
    scipy.io.savemat(file_name+'.mat', mdict={'obst_grid': obst_grid, 'targ_grid': targ_grid})
    plt.imshow(obst_grid - targ_grid, cmap='Greys_r')
    plt.show(block=False)
    plt.pause(0.01)
    plt.savefig(file_name+'.png')
    plt.close()
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml', type=str, help='path to yaml')
    parser.add_argument('-num_grids', type=int, default=10)
    args = parser.parse_args()

    assert(os.path.exists(args.yaml))

    with open(args.yaml, 'r') as f:
        descr = yaml.load(f)

    for i in xrange(args.num_grids):
        obst_grid, targ_grid = generate_grid(descr['size'], descr['density'], descr['object_density_range'], descr['object_separation'],
                             descr['goal']['center'], descr['goal']['extents'])
        save_grid(obst_grid, targ_grid, os.path.join(os.path.dirname(args.yaml), 'grid{0}'.format(i)))
        
   
