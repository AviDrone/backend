# Example missions

## Running a Basic Mission

The only difference between running a real-life mission, and
a simulation is which connection string we pass to our code.

To run a basic mission simulation, read the guide in [SIMULATION.md](SIMULATION.md)


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