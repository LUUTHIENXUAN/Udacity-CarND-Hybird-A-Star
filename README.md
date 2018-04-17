# Hybrid A* Pseudocode:

Hybrid A* is a continuous method with discrete angle resolution.  Similarly to A*, it also uses an optimistic heuristic. Because of discrete angle resolution in searching, it does not always find a solution when one exits.  The great improvement from A* is that the solution is always guaranteed to be drive-able but not optimal compared to the properties of A*.

## [1] Introduction
The pseudocode below outlines an implementation of the A* search algorithm using the bicycle model. The following variables and objects are used in the code but not defined there:

-   `State(x, y, theta, g, f)`: An object which stores  `x`,  `y`  coordinates, direction  `theta`, and current  `g`and  `f`  values. 
- Grid:  A 2D array of `0s` and `1s` indicating the area to be searched. 
   `1s` correspond to obstacles, and `0s` correspond to free space.
-   `SPEED`: The speed of the vehicle used in the bicycle model.
-   `LENGTH`: The length of the vehicle used in the bicycle model.
-   `NUM_THETA_CELLS`: The number of cells a circle is divided into. 
    This is used in keeping track of which States we have visited already.

The bulk of the hybrid A* algorithm is contained within the  `search`  function. The  `expand`  function takes a state and goal as inputs and returns a list of possible next states for a range of steering angles. This function contains the implementation of the bicycle model and the call to the A* heuristic function.

## [2] Model
Some useful [bicycle models](http://myenigma.hatenablog.com/entry/20140301/1393648106) for updating state  in `update_state` function:

<!DOCTYPE html>
<html>
<head>
  <!-- meta charset="UTF-8" -->
  <!-- meta name="viewport" content="width=device-width, initial-scale=1.0" -->
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
</head>
<body>
<table style="width:100%" >
  <tr>
    <th>
      <p align="center">
           <img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{array}{rcl}{\omega}_{t}&=&\frac{v}{L}.\tan({\delta}(t))\\x_{t+1}&=&x_{t}+v.{\Delta}t.cos{\theta}_{t}\\y_{t+1}&=&y_{t}+v.{\Delta}t.sin{\theta}_{t}\\{\theta}_{t+1}&=&{\theta}_{t}+{\Delta}t.{\omega}_{t}\\\end{array}" alt="Overview" width="62%" height="62%"></a>
           <br style="font-size:10vw;">[1] 直線運動モデル</br> 
      </p>
    </th>
    <th><p align="center">
           <img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{array}{rcl}{\omega}_{t}&=&\frac{v}{L}.\tan({\delta}(t))\\{\theta}_{t+1}&=&{\theta}_{t}+{\Delta}t.{\omega}_{t}\\x_{t+1}&=&x_{t}+v.{\Delta}t.cos{\theta}_{t+1}\\y_{t+1}&=&y_{t}+v.{\Delta}t.sin{\theta}_{t+1}\\\end{array}" alt="Overview" width="55%" height="55%"></a>
           <br style="font-size:10vw;">[2] 直線運動モデル </br>
        </p>
    </th>
  </tr>
  <tr>
   <th><p align="center">
           <img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{array}{rcl}{\omega}_{t}&=&\frac{v_{t}}{L}.\tan({\delta}(t))\\x_{t+1}&=&x_{t}+v_{t}.{\Delta}t.cos({\theta}_{t}+\frac{{\omega}_{t}.{\Delta}t}{2})\\y_{t+1}&=&y_{t}+v_{t}.{\Delta}t.sin({\theta}_{t}+\frac{{\omega}_{t}.{\Delta}t}{2})\\{\theta}_{t+1}&=&{\theta}_{t}+{\Delta}t.{\omega}_{t}\\\end{array}" alt="Overview" width="80%" height="80%"></a>
           <br style="font-size:10vw;">[3] 直線運動モデル </br>
        </p>
    </th> 
    <th><p align="center" style="font-size:10vw;">
           <img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{array}{rcl}{\omega}_{t}&=&\frac{v_{t}}{L}.\tan({\delta}(t))\\x_{t+1}&=&x_{t}-\frac{v_{t}}{{\omega}_{t}}.sin{\theta}_{t}+\frac{v_{t}}{{\omega}_{t}}.sin({\theta}_{t}+{\omega}_{t}.{\Delta}t)\\y_{t+1}&=&y_{t}-\frac{v_{t}}{{\omega}_{t}}.cos{\theta}_{t}-\frac{v_{t}}{{\omega}_{t}}.cos({\theta}_{t}+{\omega}_{t}.{\Delta}t)\\{\theta}_{t+1}&=&{\theta}_{t}+{\Delta}t.{\omega}_{t}\\\end{array}"  alt="Overview" width="90%" height="90%"></a>
           <br style="font-size:10vw;">[4] 直線運動モデル </br>
        </p>
    </th>
  </tr>
</table>
</body>
</html>

## [3] Code

```python
# Update next state of model when expanding the search
def update_state(current_x, current_y, current_theta, SPEED, LENGTH, delta):
    
    # ---Begin bicycle model--- MODEL NO.1
    delta_rad  = deg_to_rad(delta) # convert to radian
    omega      = SPEED/LENGTH * tan(delta_rad) 
    
    x          = current_x + SPEED * cos(theta)
    y          = current_y + SPEED * sin(theta)
    theta      = normalize(current_theta + omega)
    # ---End bicycle model-----
    
    return x, y, theta
```

```python
# http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#
# heuristics-for-grid-maps

### Manhattan distance D=1
def heuristic_Man(x, y, goal):
    dx = abs(x - goal.x)
    dy = abs(y - goal.y)
    return D * (dx + dy) 

### Diagonal distance
### *Chebyshev distance D = 1 and D2 = 1
### *Octile distance    D = 1 and D2 = sqrt(2)
def heuristic_Dia(x, y, goal):
    dx = abs(x - goal.x)
    dy = abs(y - goal.y)
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

### Euclidean distance
def heuristic_Euc(x, y, goal):
    dx = abs(x - goal.x)
    dy = abs(y - goal.y)
    return D * sqrt(dx * dx + dy * dy)

### Breaking ties p = 1/1000
### p < (minimum cost of taking one step)/(expected maximum path length)
def heuristic_Bre(heuristic_value):
    new_heuristic = heuristic_value*(1.0 + p) 
    return new_heuristic 
```
```python
# Update cost at new state when expanding the search
def update_cost(current_g, x, y, goal):
    g = current_g + 1
    f = g + heuristic(x, y, goal)
    return g, f
```

```python
def expand(state, goal):
    next_states = []
    for delta in range(-35, 40, 5): 
        # Create a trajectory with delta as 
        # the steering angle using the bicycle model:
        
        # Update from current state
        next_x, next_y, next_theta = update_state(state.x, state.y, state.theta,
                                                  SPEED, LENGTH, delta)
     
        # Update cost function
        next_g, next_f = update_cost(state.g, next_x, next_y, goal)  

        # Create a new State object with all of the "next" values.
        state = State(next_x, next_y, next_theta, next_g, next_f)
        next_states.append(state)

    return next_states
```
```python
def search(grid, start, goal):
    # The opened array keeps track of the stack of States objects we are 
    # searching through.
    opened = []
    
    # 3D array of zeros with dimensions:
    # (NUM_THETA_CELLS, grid x size, grid y size).
    closed = [[[0 for x in range(grid[0])] for y in range(len(grid))] for cell in range(NUM_THETA_CELLS)]
    
    # 3D array with same dimensions. Will be filled with State() objects to keep 
    # track of the path through the grid. 
    came_from = [[[0 for x in range(grid[0])] for y in range(len(grid))] for cell in range(NUM_THETA_CELLS)]

    # Create new state object to start the search with.
    x = start.x
    y = start.y
    theta = start.theta
    g = 0
    f = heuristic(start.x, start.y, goal)
    state = State(x, y, theta, 0, f)
    opened.append(state)

    # The range from 0 to 2pi has been discretized into NUM_THETA_CELLS cells. 
    # Here, theta_to_stack_number returns the cell that theta belongs to. 
    # Smaller thetas (close to 0 when normalized  into the range from 0 to 2pi) 
    # have lower stack numbers, and larger thetas (close to 2pi whe normalized)
    # have larger stack numbers.
    stack_number = theta_to_stack_number(state.theta)
    closed[stack_number][index(state.x)][index(state.y)] = 1

    # Store our starting state. For other states, we will store the previous state 
    # in the path, but the starting state has no previous.
    came_from[stack_number][index(state.x)][index(state.y)] = state

    # While there are still states to explore:
    while opened:
        # Sort the states by f-value and start search using the state with the 
        # lowest f-value. This is crucial to the A* algorithm; the f-value 
        # improves search efficiency by indicating where to look first.
        opened.sort(key=lambda state:state.f)
        current = opened.pop(0)

        # Check if the x and y coordinates are in the same grid cell as the goal. 
        # (Note: The idx function returns the grid index for a given coordinate.)
        if (idx(current.x) == goal[0]) and (idx(current.y) == goal.y):
            # If so, the trajectory has reached the goal.
            return path

        # Otherwise, expand the current state to get a list of possible next states.
        next_states = expand(current, goal)
        
        for next_state in next_states:
            
            # If we have expanded outside the grid, skip this next_state.
            if next_states is not in the grid:
                continue
            
            # Otherwise, check that we haven't already visited this cell and
            # that there is not an obstacle in the grid there.
            stack_number = theta_to_stack_number(next_state.theta)
            
            if closed_value[stack_number][idx(next_state.x)][idx(next_state.y)] == 0 and
               grid[idx(next_state.x)][idx(next_state.y)] == 0:
            
                # The state can be added to the opened stack.
                opened.append(next_state)
                
                # The stack_number, idx(next_state.x), idx(next_state.y) tuple 
                # has now been visited, so it can be closed.
                closed[stack_number][idx(next_state.x)][idx(next_state.y)] = 1
                
                # The next_state came from the current state, and that is recorded.
                came_from[stack_number][idx(next_state.x)][idx(next_state.y)] = current

```
