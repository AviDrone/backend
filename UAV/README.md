# UAV

## calibration.py

``` {python}
async def calibrate():
```

### Description

Calibrates the following UAV sensors:

- Gyroscope
- Accelerometer
- Magnetometer
- Board level

## gps_data.py

### add_point(self, new_gps_point, new_distance)

``` {python}
def add_point(self, new_gps_point, new_distance):
```

Insert new_gps_point and new_direction_distance
into the beginning of the list. And trim list to
fit the window_size.

### get_minimum_index(self)

``` {python}
def get_minimum_index(self):
```

Return the minimum distance index within the window

### purge_gps_window(self)

minimum_distance_index

``` {python}
purge_gps_window(self):
```

Clears the gps_points array

### parameters.py

``` {python}
def parameter()
```

Initializes parameters sent to the drone

#### Returns

list with the parameters at each index in the following order:

- MAGNITUDE, Set the distance the drone goes
- HEIGHT, Set the height of the flight path
- DEGREES, Set the amount to rotate in yaw
- DEGREE_ERROR, Number of degrees error for rotation
- DISTANCE_ERROR, Error in distance before target reached
- LAND_THRESHOLD , Error in distance before target reached
- WINDOW_SIZE, Set the

## telemetry.py

Prints the following information:

- Battery
- GPS_info
- In_air
- Position


## Sources

[Flight Modes](https://docs.px4.io/master/en/config/flight_mode.html)

[Flight Modes DOCS](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html)
