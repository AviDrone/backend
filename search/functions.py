# TODO refactor


def simple_goto_wait(goto_checkpoint) -> None:
    vehicle.simple_goto(goto_checkpoint)

    distance = better_get_distance_meters(get_global_pos(), goto_checkpoint)

    while distance >= DISTANCE_ERROR and vehicle.mode.name == "GUIDED":
        print(distance)
        distance = better_get_distance_meters(get_global_pos(), goto_checkpoint)
        time.sleep(1)

    if vehicle.mode.name != "GUIDED":
        vehicle.simple_goto(vehicle.location.global_frame)
        print("Halting simple_goto")

    print("Checkpoint reached")

    def condition_yaw(heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.
        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behavior (https://github.com/diydrones/ardupilot/issues/2427)
        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        """
        Found this here:
        https://github.com/dronekit/dronekit-python/blob/master/examples/guided_set_speed_yaw/guided_set_speed_yaw.py
        Modified it to allow for clockwise and counter-clockwise operation
        """
        original_yaw = vehicle.attitude.yaw
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle

        if heading < 0:
            heading = abs(heading)
            cw = -1
        else:
            cw = 1

        # create the CONDITION_YAW command using command_long_encode()
        msg = vehicle.message_factory.command_long_encode(
            0,  # target system
            0,  # target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            cw,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0,
            0,
            0,
        )  # param 5 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)

        """
        Adding additional code to make drone wait till the wobbling settles down
        before moving forward.
        """
        heading_rad = heading * math.pi / 180
        target_yaw = original_yaw + cw * heading_rad

        # 1 Degree of error....
        while abs(target_yaw - vehicle.attitude.yaw) % math.pi > 0.01745 * DEGREE_ERROR:
            print("Turn error: ", abs(target_yaw - vehicle.attitude.yaw) % math.pi)
            time.sleep(0.25)

    # if time allows

    def forward_calculation():
        flight_direction = []
        yaw = vehicle.attitude.yaw

        print(yaw)
        flight_direction.append(MAGNITUDE * math.cos(yaw))
        flight_direction.append(MAGNITUDE * math.sin(yaw))

        return flight_direction
