## Udacity FCND - Project 2 - Writeup
Author: Tamas Panyi
ver: 1

### The Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
This is how the motion planning works in the solutuoin.

1. The the map colliders are read into memory from the `colliders.csv` file.
1. All the state transitions (function pospend with `_transition`) are provided ready to use.
1. Functions to convert between global and local coordinates
1. Drone driver in (`drone.py`) that comminucates with and handles the drone.
1. Grid creation (`create_grid`) from collider data and the crusing altitude of the drone, taking also a safety distance around the obstacles' peremiter into account.
1. Helper function to compute valid actions for the provided grid position given the grid. This was upgraded in my solution to handle eight planar directions, extending the originally provided four.
1. A* path finder algorithm implementation.


### Implementing Your Path Planning Algorithm

#### 1. Set global home position
The global home is obtained from the first line of the provided csv file: 
```python
lat0 = float(re.search(r"(?<=lat0 )(.+)(?=,)", line).group(0).strip())
lon0 = float(re.search(r"(?<=lon0 )(.+)", line).group(0).strip())
```
These are then set as the global home position of the drone.
```python
self.set_home_position(lon0, lat0, 0.0)
```
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


#### 2. Set current local position
The local position is calculated from the global one like so:
```python
local_position =tuple([int(x) for x in global_to_local(self.global_position, self.global_home)])
```
The grid represenation's domain is integers, hence the int conversion in the above formula.

#### 3. Set grid start position from local position
The grid start position is related to the local position:
```python
grid_start = (local_position[0] - north_offset, local_position[1] - east_offsset)
```

#### 4. Set grid goal
Gvien that the plot is small in size the wiggle room is really small, hence setting the goal by global coordinates is impractical, so I removed it.
The goal can be specifed by grid coordinates. The method `set_valid_goal_around_target` will look for a suitable landing spot in the vicinity of the provided goal until it finds one.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I sticed with the A* algorithm due to its comparatvie good performance.
Dijkstra's algorithm would be a viable alternative, although the fact that A* utilizes a heuristic it will be in most cases find the shortest way faster.  
The list of valid actions are extended to include diagonal movements that are weighed with their corresponding planar distance, \sqrt{2}

#### 6. Cull waypoints 
The unnecessary waypoints are pruned by utilizing a collinearity check, see the `prune` method in `planning_utils.py`.

The method is implemented as a generator function to effctively defer execution thus enumerating the collection of points only once. This is to alleviate the fact that python is - generally speaking - slow. 
See `collinearity_check`; tt checks each point, except for the extremes, and assigns the boolian value of `True` to them if theye are collinear to their neighbours. The resulting list of booleans is used to construct a list that only includes non collinear waypoints:
```python
new_path = [ p for p, c in zip(path, collinearity_check(path, epsilon)) if not c ]
```
This method is not able to cull the sometimes unnecessary wigling when the drone traverses a distance in the likness of a knigh's move (L-shape). 



