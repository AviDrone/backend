# Simulation

## Installation
To run a sim, make sure to have mavproxy, dronekit, dronekit-sitl, and qgroundcontrol installed

Example installation command with pip for the first three:
> pip install dronekit dronekit-sitl mavproxy

QGroundControl can be installed here: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## To Setup Dronekit-SITL:
Open a terminal, use the following command. Modify the coordinates, altitude, and angle as needed.
> dronekit-sitl copter --home=46.045577,-118.391351,584,127

(Format: dronekit-sitl copter --home=lat,long,alt,angle)

In another terminal, run:
> mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552

If this does not work on Windows, try setting this command up on a Ubuntu terminal. There are also installation tutorials available on YouTube.

## Connect in QGroundControl
In QGroundControl, click the icon in the upper right corner. Select:

Application settings -> Comm links 

setup a UDP connection by clicking "add" choose "UDP" with a value of 14550 for port and an appropriate name for the connection. Hit connect. The two commands above need to be running in terminals to get a successful connection.

## Run a script
Navigated to the desired directory (with your script) in a third terminal, run:

> python insertScriptHere.py --connect udp:127.0.0.1:14551

or

> python3 insertScriptHere.py --connect udp:127.0.0.1:14551

or

> py -3.8 insertScriptHere.py --connect udp:127.0.0.1:14551


"insertScriptHere.py" refers to the python file you wish to run. For example, primary.py in the directory 
...\avidrone\search\primary.py


<!--TRANSFERING NATE'S INSTRUCTIONS TO HERE>
