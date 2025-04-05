import time as t

# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from simu import imu


def driveXMeters(x=1.0, vel=0.2):
    """
    driveXMeters(x=1.0) - drive x meters forward
    x = distance in meters to drive forward
    """
    state = 0
    pose.tripBreset()
    print(f"% Driving {x}m -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            vel = vel if x > 0 else -vel
            vel_cmd = f"{vel:.2f} 0.0"  # (forward m/s, turn-rate rad/sec)
            service.send(
                "robobot/cmd/ti/rc", vel_cmd
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            if abs(pose.tripB) > abs(x) or pose.tripBtimePassed() > 15:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            print(
                f"# drive {x}m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print(f"% Driving {x}m ------------------------- end")


def driveUntilWall(d=0.2, ir_id=1):
    """
    driveUntilWall(d=0.2) - drive until wall is detected
    d = distance to the wall in meters to stop at
    ir_id = IR sensor id (0 or 1), 0: right, 1: front
    """
    min_d = 1.5
    state = 0
    pose.tripBreset()
    print(f"% Driving until wall is at {d}m -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            service.send(
                "robobot/cmd/ti/rc", "0.2 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            if ir.ir[ir_id] < d or pose.tripBtimePassed() > 15:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
                min_d = ir.ir[ir_id]
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            print(
                f"# drive drove {pose.tripB:.3f}m. Stopped at {ir.ir[ir_id]:.3f}m from the wall. {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# drive {state}, ir: {ir.ir}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print("% Driving until wall ------------------------- end")
    return min_d


def driveUntilLine(threshold=300):
    """
    driveUntilLine() - drive until line is detected
    """
    state = 0
    pose.tripBreset()
    print(f"% Driving until line -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            service.send(
                "robobot/cmd/ti/rc", "0.2 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            line_sensor = edge.edge_n
            line_sensor = [abs(s) for s in line_sensor]  # absolute value
            line_sensor = max(line_sensor)  # max of all 3 axes
            if line_sensor > threshold or pose.tripBtimePassed() > 30:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            print(
                f"# drive drove {pose.tripB:.3f}m. Stopped at line values {edge.edge_n}. {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# drive {state}, line: {edge.edge_n}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print("% Driving until line ------------------------- end")


def climbCircle(acc=50, vel=0.5):
    """
    driveUntilWall(d=0.2) - drive until a certain acceleration is detected
    acc = acceleration in m/s^2 to stop at
    """
    max_acc = 0.0
    state = 0
    pose.tripBreset()
    print(f"% Driving until acc spike of {acc} m/s2 -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            service.send(
                "robobot/cmd/ti/rc", f"{vel} 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            gyro = [imu.gyro[0], imu.gyro[1], imu.gyro[2]]
            gyro = [abs(g) for g in gyro]  # absolute value
            gyro = max(gyro)  # max of all 3 axes
            if gyro > acc or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti/rc", "0.1 0.0")
                t.sleep(2.5)  # wait for stop
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
                max_acc = gyro
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            print(
                f"# Max acc = {max_acc}, drive drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# drive {state}, acc {imu.acc}, gyro {imu.gyro} now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print("% Driving until wall ------------------------- end")


def turnInPlace(deg=90, dir=0, ang_speed=0.8):
    """
    NEEDS CALIBRATION (NOT ACCURATE)
    "turnInPlace(rad=3.14) - turn in place rad radians"
    deg = angle in degrees to turn in place
    dir = 0: counter-clockwise, 1: clockwise
    """
    rad = deg * 3.14 / 180.0
    state = 0
    pose.tripBreset()
    print(f"% Turning {deg} degrees -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            ang_speed = ang_speed if dir == 0 else -ang_speed
            cmnd_msg = f"0.0 {ang_speed:.2f}"
            service.send(
                "robobot/cmd/ti/rc", cmnd_msg
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            if abs(pose.tripBh) > rad or pose.tripBtimePassed() > 15:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                state = 99
        else:
            print(
                f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# turn {state}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print(f"% Truning {rad} rad ------------------------- end")


from collections import deque


def orientateToWall(ir_id=1, dir=0, tolerance=0.01, window=5, timeout=10):
    """
    Rotate until the IR distance starts increasing (after decreasing), using a moving average.

    ir_id: which IR sensor (0 = right, 1 = front)
    dir: 0 = counter-clockwise, 1 = clockwise
    tolerance: minimum increase to detect a trend reversal
    window: number of samples for moving average
    timeout: max time in seconds to attempt before stopping
    """
    state = 0
    pose.tripBreset()
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")
    print("% Starting orientation to wall with noise filtering")

    ir_history = deque(maxlen=window)
    min_avg = float("inf")

    while not service.stop:
        if state == 0:
            turn_rate = 0.5 if dir == 0 else -0.5
            service.send("robobot/cmd/ti/rc", f"0.0 {turn_rate}")
            state = 1

        elif state == 1:
            ir_value = ir.ir[ir_id]
            ir_history.append(ir_value)

            if len(ir_history) == window:
                avg = sum(ir_history) / window
                if avg < min_avg:
                    min_avg = avg
                elif avg > min_avg + tolerance:
                    print(
                        f"% Detected increase in IR avg: {avg:.3f} > {min_avg + tolerance:.3f}"
                    )
                    service.send("robobot/cmd/ti/rc", "0.0 0.0")
                    state = 2

                print(f"# IR avg: {avg:.3f}, min_avg: {min_avg:.3f}")

        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                state = 99

        else:
            print(
                f"# Finished turning {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} sec, min IR avg: {min_avg:.3f}"
            )
            break

        if pose.tripBtimePassed() > timeout:
            print("% Timeout — stopping")
            service.send("robobot/cmd/ti/rc", "0.0 0.0")
            break

        t.sleep(0.05)

    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    print("% Orientation to wall complete")


def followWall(d=0.3, velocity=0.2, time=60.0, d_front=0.1, Kp=1.3, Ki=0.0, Kd=2.0):
    """
    followWall(d=0.3) - follow wall on the right side at distance d
    Uses PID control and rotates in place to recover wall when lost.
    d = distance to the wall in meters to follow
    velocity = forward velocity in m/s
    time = time in seconds to follow the wall
    Kp = proportional gain
    Ki = integral gain
    Kd = derivative gain
    """
    state = 0
    pose.tripBreset()
    integral = 0.0
    last_error = 0.0
    filtered_ir = ir.ir[0]  # initial smoothed value
    print(f"% Following wall at {d}m -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 0 100")  # blue

    while not service.stop:
        if ir.ir[1] < d_front:
            print("# Crash detected, stopping.")
            service.send("robobot/cmd/ti/rc", "0.0 0.0")
            break
        # --- Sensor smoothing ---
        raw_distance = ir.ir[0]
        filtered_ir = 0.8 * filtered_ir + 0.2 * raw_distance
        distance = filtered_ir

        # --- Wall loss recovery ---
        if distance > 0.6 or distance < 0.1:
            print(f"# Wall lost (IR={distance:.3f}m), rotating to find wall...")
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.5"
            )  # rotate counterclockwise (left)
            while not service.stop:
                raw_distance = ir.ir[0]
                filtered_ir = 0.8 * filtered_ir + 0.2 * raw_distance
                distance = filtered_ir

                print(f"# Searching... IR={distance:.3f}m")
                if 0.1 < distance < 0.5:
                    print("# Wall reacquired, resuming wall following.")
                    pose.tripBreset()
                    break
                t.sleep(0.05)
            continue  # skip PID and resend velocity after reacquiring

        # --- PID control ---
        error = max(min(d - distance, 0.3), -0.3)  # Clamp error
        if abs(error) < 0.01:
            error = 0.0  # Deadband

        integral += error * 0.05
        derivative = (error - last_error) / 0.05
        last_error = error

        turn = Kp * error + Ki * integral + Kd * derivative
        turn = max(min(turn, 1.0), -1.0)  # Clamp

        if state == 0:
            service.send("robobot/cmd/ti/rc", f"{velocity:.2f} {turn:.2f}")
            print(
                f"# Wall follow: IR={distance:.3f}m, error={error:.3f}, turn={turn:.3f}"
            )
            if pose.tripBtimePassed() > time:
                state = 1
        elif state == 1:
            service.send("robobot/cmd/ti/rc", "0.0 0.0")
            if abs(pose.velocity()) < 0.001:
                break

        t.sleep(0.05)

    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    print("% Wall following ------------------------- end")


def rotateCircle(r=0.5, deg=360, dir=0):
    """
    rotateCircle() - Drive in a circle with a specified radius.
    r = radius in meters
    deg = angle to rotate in degrees (default 360 for full circle)
    dir = 0: counter-clockwise, 1: clockwise
    """
    from spose import pose
    from uservice import service
    import time as t

    state = 0
    rotation = 0.0
    pose.tripBreset()
    print(f"% Driving in a circle with radius {r}m -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green LED

    while not service.stop:
        if state == 0:
            # Send circular movement command: forward speed = 0.2 m/s, angular speed = 0.2 / r
            r = r if dir == 0 else -r  # Adjust radius based on direction
            vel_cmd = f"0.2 {0.2 / r:.2f}"
            service.send("robobot/cmd/ti/rc", vel_cmd)
            state = 1

        elif state == 1:
            # Compute rotated angle in degrees
            rotation = (pose.tripB / r) * (180 / 3.1416)
            if abs(rotation) > deg or pose.tripBtimePassed() > 30:
                # Stop if completed circle or timeout
                service.send("robobot/cmd/ti/rc", "0.0 0.0")
                state = 2

        elif state == 2:
            # Wait until the robot fully stops
            if abs(pose.velocity()) < 0.001:
                state = 99

        elif state == 99:
            break

        # Logging for debugging
        print(
            f"# drive {state}, rot {rotation:.3f}, dist {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} sec"
        )
        t.sleep(0.05)

    # Stop everything and reset LED
    service.send("robobot/cmd/ti/rc", "0.0 0.0")
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # turn LED off
    print(f"% Driving in a circle ------------------------- end")
