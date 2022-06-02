# Simulation

>**Note:** You will need to have mavproxy, dronekit, dronekit-sitl, and qgroundcontrol installed

## Commands to run for simulation

Open two separate terminals and qgroundcontrol.

In the first terminal,  run

```{bash}
dronekit-sitl copter
```

in the second terminal, run

```{bash}
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552

```

>**Note:** You will need to be in the MAVproxy/MAVproxy directory to run this file. You might need to clone the [MAVproxy](https://github.com/ArduPilot/MAVProxy) repository to your machine

Set **dronekit-sitl copter** to a test location.

## Test locations

## Default dronekit test location

```{bash}
dronekit-sitl copter --home=46.045577,-118.391351,584,353
```

or, to test over WWU, use the following locations:

### Roger's field

```{bash}
dronekit-sitl copter --home=46.045577,-118.391351,584,353
```

### Kretchmar

```{bash}
dronekit-sitl copter --home=46.045577,-118.391351,584,127
```

In QGroundControl:
Application settings -> Comm links -> connect using "sim" (which is setup using a UDP connection)

## Run a script

- Open a third terminal
- Navigate to your mission's directory
- run the following command below
  
```{bash}
python3 <insertScriptHere.py> --connect udp:127.0.0.1:14551
```

## Example

Get started by trying the following mission

```{bash}

python3 modified_mission_basic.py --connect udp:127.0.0.1:14551
```
