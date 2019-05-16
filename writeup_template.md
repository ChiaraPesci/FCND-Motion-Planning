## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes a sequence of actions the quad will follow.

In `motion_planning.py` the sequence of commands is provided, and it reads
1. Initialization and MANUAL state activated
2. Being in MANUAL state, the `arming_transition()` is called. In this transition the drone is armed and the controls are given to the computer pilot. The state is changed to ARMING.
3. In ARMING state and after the check that the drone is armed, the path is computed in *plan_path()* and the waypoints stored in the list `self.waypoints`. In details, *plan_path()* prescribes the following commands
  - Change to PLANNING state
  - Set the target altitude as new target position
  - Set the safety distance
  - Load the obstacle map and create the grid accordingly with the aid of the function *crate_grid()* in `planning_utils.py`
  - Set start and goal positions (the offset with respect to the bottom left corner of the grid has to be taken into account)
  - Find a path via *a_star()* function in `planning_utils.py` given the grid, an heuristic (defined in `planning_utils.py`, too) and initial and goal positions
  - Convert the path to waypoints (the offset with respect to the bottom left corner of the grid has to be taken into account here, too) to be saved in `self.waypoints`
  - The waypoints are "sent" to the simulator to be visualized


4. In PLANNING state, takeoff can take place, i.e. the function *takeoff_transition()* is executed where the state is changed to TAKEOFF and the quad takes off up to the altitude prescribed above (z-component of target_position), `self.takeoff(self.target_position[2])`
5. In TAKEOFF state, when the vertical position of the drone is in a 5% range around the target altitude, the *waypoint_transition()* can start
6. [Iterate for all waypoints] The state is changed to WAYPOINT. The new target position is set to the first element of the list `self.waypoints` with the method `list.pop(index)`. This method, with index=0, returns the first element of the list and it removes it from the list. The drone is commanded to this new target position via the function *cmd_position()*.
7. [Iterate for all waypoints] Once the commanded position is reached (the drone is within 1 m range the target position), if the `self.waypoints` list is not empty then the new target position is set. Point 6. and 7. are repeated.
8. If any waypoint is left in the list `self.waypoints` and the drone velocity magnitude is less than a certain threshold, *landing_transition()* can occur. The state is changed to LANDING and the drone is commanded to land via *land()* function
9. In LANDING state, the *velocity_callback()* is called and when the quad fulfills the constraints on the position, the *disarming_transition()* can start. Here the state is changed to DISARMING, the rotors are switched off *disarm()* and the controls are back to manual *release_control()*

In `planning_utils.py` the functions used in the steps above are implemented.
- *create_grid()* generates a 2D grid, given the position of the obstacles at a certain altitude and taking into account a safety margin around the obstacles. This function should be extended to return a 2.5D grid which stores the obstacle height
- In the class *Actions*, the possible drone actions are defined as north-east movements with relative cost. The function *valid_actions()* returns a list of valid actions given a grid and the current node. The class has to be extended to allow for diagonal motion
- *a_star()* returns a path, if found, given a grid, a heuristic (in this case Euclidian distance), start and goal positions. The function evaluate each possible action in the path according to feasibility and costs.
- The *heuristic(position, goal_position)* function return the Euclidian norm between the current position and the goal position


The main difference to the `backyard_flyer` is in the path planning. In the first project the waypoints were set by the user, hard-coded, while here the waypoints are computed after the path evaluation between points A and B.

<!-- And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd -->

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

    # Read lat0, lon0 from colliders into floating point values
    filename = 'colliders.csv'
    readFirstLine = open(filename).readline().split()
    # print(readFirstLine)
    lat0 = float(readFirstLine[1].strip(','))
    lon0 = float(readFirstLine[3])
    print('Global home lon and lat: {0}, {1}'.format(lon0, lat0))

    # Set home position to (lon0, lat0, 0)
    self.set_home_position(lon0, lat0, 0.0)


<!-- And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png) -->

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

    # Retrieve current global position
    global_position = (self.longitude, self.latitude, self.altitude)
    print('Global position: {0}'.format(global_position))

    # Convert to current local position using global_to_local()
    local_position = global_to_local(global_position, self.global_home)
    print('Local position: {0}'.format(local_position))


<!-- Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png) -->

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

    grid_start = (int(local_position[0])-north_offset,
                  int(local_position[1])-east_offset)

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

    goal = (grid_start[0] + 10, grid_start[1] - 12, 0.0)
    grid_goal_global = local_to_global(goal, self.global_home)
    grid_goal = (goal[0], goal[1])
    print('Global goal: {0}, local grid goal: {1}'.format(grid_goal_global, grid_goal))

This task is not fully clear to me. I could not understand how to pass from the geodetic reference system to the one of the map.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

In the class *Actions* the following movements have been added:
- NE = north-east
- NW = north-west
- SE = south-east
- SW = south-west

Then in the *valid_actions()* function these movements are checked if possible or not, i.e. they would make the quad collide with an obstacle. If the move is not allowed, then it is removed from the list of valid actions.

#### 6. Cull waypoints
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

To prune the path I added the following functions from the exercises:
- *point()*: given a 2D point it returns a 3-components vector with z=1 necessary to compute the determinant in the collinearity check
- *collinearity_check()*: function that given three points it checks if they are collinear or not, it returns a boolean (True if collinear)
- *prune_path()*: if three points along the path are collinear, remove the point in the middle. Check performed for all the points triplets



### Execute the flight
#### 1. Does it work?
It works!
![Top View](./misc/P2_fig1.png)

![Closer View](./misc/P2_fig2.png)

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
