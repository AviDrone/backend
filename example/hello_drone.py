#!/usr/bin/python
import dronekit_sitl
from dronekit import connect

print("Start simulator (SITL)")
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print("Vehicle State:")
print(" Battery: %s" % vehicle.battery)
print(" Is armable?: %s" % vehicle.is_armable)
print(" Mode: %s" % vehicle.mode.name)  # settable
print(" GPS: %s" % vehicle.gps_0)
print(" System status: %s" % vehicle.system_status.state)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")
