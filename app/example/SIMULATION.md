# Simulation

## Installation

>**Note:** To run a simulation, make sure to have mavproxy, dronekit, dronekit-sitl, and qgroundcontrol installed.

An installation guide for QGroundControl can be found in their [download and install](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html) guide.

## To Setup Dronekit-SITL

Open a terminal, use the following command. Modify the coordinates, altitude, and angle as needed.

### Test locations

#### Default dronekit test location

```{bash}
dronekit-sitl copter --home=46.045577,-118.391351,584,353
```

or, to test over WWU, use the following locations:

#### Roger's field

```{bash}
dronekit-sitl copter --home=46.045577,-118.391351,584,353
```


```{bash}
dronekit-sitl copter --home=46.0452822,-118.3930353,584,180
```


or (with a different angle)

```{bash}
dronekit-sitl copter --home=46.045577,-118.391351,584,127
```

>**Format:** dronekit-sitl copter --home=lat,long,alt,angle

In another terminal, run:

```{bash}
cd MAVproxy/MAVproxy && python3 mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552
```

>**Note:** You will need to be in the MAVproxy/MAVproxy directory to run this script. You might need to clone the [MAVproxy](https://github.com/ArduPilot/MAVProxy) repository to your machine

For more in-depth, there are also installation tutorials available on YouTube. Here's a good [example](https://www.youtube.com/watch?v=h5vAjbsNUV8&ab_channel=PatchNFielder).

## Connect in QGroundControl

In QGroundControl, click the icon in the upper right corner. Select:

Application settings -> Comm links

setup a UDP connection by clicking "add" choose "UDP" with a value of 14550 for port and an appropriate name for the connection. Hit connect.

>**Note:** The two commands above need to be running in terminals to get a successful connection.

## Run a script

Navigate to the desired directory (with your script) in a third terminal, run:

### Python 2.7.x

> python <insertScriptHere.py> --connect udp:127.0.0.1:14551

or

### Python3.8.x

> python3 <insertScriptHere.py> --connect udp:127.0.0.1:14551

or

> py -3.8 <insertScriptHere.py> --connect udp:127.0.0.1:14551

"insertScriptHere.py" refers to the python file you wish to run. For example, **primary.py** in the directory
...\avidrone\search\primary.py
