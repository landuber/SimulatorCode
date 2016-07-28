#!/usr/bin/python
 
import numpy as np
import yaml;
 
 
 
def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position
    Output: a tuple (path, numopened)
            path: list of the indices of the nodes on the shortest path found
                starting with "start" and ending with "end" (each node is in
                  metric coordinates)
    """
    # occupancy_map = np.flipud(occupancy_map)
    shape = occupancy_map.shape
    occupied = occupancy_map == 1
    free = occupancy_map == 0
    o_start = start
    o_goal = goal
    start = (int(start[1][0]//y_spacing), int(start[0][0]//x_spacing))
    goal = (int(goal[1][0]//y_spacing), int(goal[0][0]//x_spacing))
    # map to track the visitation state of each grid element
    state_map = np.zeros(occupancy_map.shape)
    state_map[free] = 1
    state_map[occupied] = 2
    state_map[start] = 5
    state_map[goal] = 6
 
    # map to track of the parent of each grid element
    parent = np.zeros(shape, dtype=np.int)
 
    # map to track the distance of each grid element from start
    distanceFromStart = np.full(shape, np.inf)
 
    # set distanceFromStart of start to 0
    distanceFromStart[start] = 0
 
    # keep track of number of nodes expanded
    numExpanded = 0
    while True:
        current = np.unravel_index(np.argmin(distanceFromStart), shape)
        # Bail out if we reached our goal or can't reach the goal
        min_dist = distanceFromStart[current]
        if current == goal or np.isinf(min_dist):
            break
        state_map[current] = 3 # mark current node as visisited
        numExpanded = numExpanded + 1
        distanceFromStart[current] = np.inf # remove this node from further consideration
        
 
 
        # neighbors = np.unravel_index(
        #            [np.ravel_multi_index(np.array(current) + np.array((1, 0)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((-1,0)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((0, 1)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((0, -1)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((1, 1)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((1, -1)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((-1, 1)), shape, mode='clip'),
        #             np.ravel_multi_index(np.array(current) + np.array((-1, -1)), shape, mode='clip')], shape)

        neighbors = np.unravel_index(
                [np.ravel_multi_index(np.array(current) + np.array((1, 1)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((1, -1)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((-1, 1)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((-1, -1)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((1, 0)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((-1,0)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((0, 1)), shape, mode='clip'),
                    np.ravel_multi_index(np.array(current) + np.array((0, -1)), shape, mode='clip')], shape)
                    

        neighbors = map(tuple, np.transpose(neighbors))
        neighbors = [item for item in neighbors if item != current]
 
        for n in neighbors:
            state = state_map[n]
            if (state != 2 and state != 3 and state != 5):
                if (distanceFromStart[n] > min_dist + 1):
                    distanceFromStart[n] = min_dist + 1
                    parent[n] = np.ravel_multi_index(current, shape)
                    state_map[n] = 4
 
 
    # Construct route from start to goal by following the parent links
    if (np.isinf(distanceFromStart[goal])):
        return np.array([])
    else:
        route = np.array([goal])
        while (parent[(route[0][0], route[0][1])] != 0):
            r = np.unravel_index(parent[(route[0][0], route[0][1])], shape)
            route = np.insert(route, 0, r, 0)

    print route

    route = [(round((r[1] + 0.5) * x_spacing, 3), round((r[0] + 0.5) * y_spacing, 3)) for r in route]
    route = np.insert(route, 0, [o_start[0][0], o_start[1][0]], 0)
    route = np.vstack([route, [o_goal[0][0], o_goal[1][0]]])

    return route
 
def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)
    s = 0
    for i in range(len(true_path1)-1):
      s += np.sqrt((true_path1[i][0]-true_path1[i+1][0])**2 + (true_path1[i][1]-true_path1[i+1][1])**2)
    print("True Path 1 length:")
    print(s)
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)
    s = 0
    for i in range(len(true_path2)-1):
      s += np.sqrt((true_path2[i][0]-true_path2[i+1][0])**2 + (true_path2[i][1]-true_path2[i+1][1])**2)
    print("True Path 2 length:")
    print(s)
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")


def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)


 
 
def main():
    test()
    # test_for_grader()
 
# def main():
#     # Load parameters from yaml
#     param_path = 'params.yaml' # rospy.get_param("~param_path")
#     f = open(param_path,'r')
#     params_raw = f.read()
#     f.close()
#     params = yaml.load(params_raw)
#     # Get params we need
#     occupancy_map = np.array(params['occupancy_map'])
#     pos_init = np.array(params['pos_init'])
#     pos_goal = np.array(params['pos_goal'])
#     x_spacing = params['x_spacing']
#     y_spacing = params['y_spacing']
#     path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
#     print(path)
 
if __name__ == '__main__':
    main()
 
