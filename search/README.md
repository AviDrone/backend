# Search

## functions.py

### Param Function
``` {python}
def param()
```

#### Description
Initializes parameters sent to the drone 


#### What the function Returns
Returns list with the parameters at each index in the following order:
- MAGNITUDE, Set the distance the drone goes
- HEIGHT, Set the height of the flight path
- DEGREES, Set the amount to rotate in yaw
- DEGREE_ERROR, Number of degrees error for rotation
- DISTANCE_ERROR, Error in distance before target reached
- LAND_THRESHOLD , Error in distance before target reached
- WINDOW_SIZE, Set the size of the gps window original: 5