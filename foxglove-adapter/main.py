import serial
import datetime
import foxglove
from foxglove import Channel
from foxglove.channels import Vector2Channel, LogChannel
from foxglove.schemas import Timestamp, Vector2, Log

SERIAL_PORT = "/dev/cu.usbmodem01"

def serial_setup():
    print("Connecting to serial port:", SERIAL_PORT)
    while True:
        try:
            ser = serial.Serial(port=SERIAL_PORT, baudrate=115200, timeout=.1)
            print("Connected to serial port:", SERIAL_PORT)
            return ser
        except serial.SerialException:
            continue

file_name = f"recordings/recording_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.mcap"

with foxglove.open_mcap(file_name):
    print("Recording at:", file_name)
    server = foxglove.start_server()

    log_channel = LogChannel("/esp_logs")
    pid_data_channel = Channel("/prev_pid", message_encoding="json")

    dprg_nav_current_pos_channel = Vector2Channel("/navigate/position/current")
    dprg_nav_target_pos_channel = Vector2Channel("/navigate/position/target")
    sense_location_channel = Channel("/sense_location", message_encoding="json")

    # Navigation messages from the firmware (navigate())
    navigate_channel = Channel("/navigate", message_encoding="json")

    locate_target_channel = Channel("/locate_target", message_encoding="json")
    esp = serial_setup()

    def read_and_log_serial():
        global esp
        while True:
            try:
                line = esp.readline()
                if line:
                    message = line.decode(errors="replace").strip()
                    if message.endswith("**"):
                        handle_possible_pid_log(message[:-2])
                    elif message.startswith("SENSE_LOCATION") and message.endswith(";"):
                        handle_possible_dprg_nav_log(message[len("SENSE_LOCATION"):-1])
                    elif message.startswith("LOCATE_TARGET") and message.endswith(";"):
                        handle_possible_locate_target_log(message[len("LOCATE_TARGET"):-1])
                    elif message.startswith("NAVIGATE") and message.endswith(";"):
                        print("NAVIGATE log", message)
                        handle_possible_navigate_log(message[len("NAVIGATE"):-1])
                    else:
                        log_channel.log(Log(message=message, timestamp=Timestamp.now()))
            except Exception as e:
                print({"message": f"Error reading serial: {str(e)}. Reconnecting..."})
                esp.close()
                esp = serial_setup()

    def handle_possible_locate_target_log(message: str):
        split = message.split(",")
        if len(split) == 4:
            try:
                parsed = [float(s) for s in split]
                xd, yd, target_distance, target_angle = parsed
                locate_target_channel.log({
                    "xd": xd,
                    "yd": yd,
                    "target_distance": target_distance,
                    "target_angle": target_angle
                })
            except ValueError:
                print("Failed to parse locate_target log", message)
    
    def handle_possible_dprg_nav_log(message: str):
        split = message.split(",")
        if len(split) == 13:
            try:
                parsed = [float(s) for s in split]
                X, X_target, Y, Y_target, theta, distance, left_inches, right_inches, deltaTime, leftTicks, rightTicks, leftEncoderTickDelta, rightEncoderTickDelta = parsed
                dprg_nav_current_pos_channel.log(Vector2(x=X, y=Y))
                dprg_nav_target_pos_channel.log(Vector2(x=X_target, y=Y_target))
                sense_location_channel.log({
                    "theta": theta,
                    "distance": distance,
                    "left_inches": left_inches,
                    "right_inches": right_inches,
                    "deltaTime": deltaTime,
                    "leftTicks": leftTicks,
                    "rightTicks": rightTicks,
                    "leftEncoderTickDelta": leftEncoderTickDelta,
                    "rightEncoderTickDelta": rightEncoderTickDelta,
                })
            except ValueError:
                print("Failed to parse dprg_nav log", message)

    def handle_possible_pid_log(message: str):
        split = message.split(",")
        if len(split) == 9:
            try:
                parsed = [float(s) for s in split]
                leftVelocityTarget, leftVelocityMeasured, _, rightVelocityTarget, rightVelocityMeasured, _, velocityOffsetFromHeading, headingTarget, currentHeading = parsed
                pid_data_channel.log({
                    "leftEncoder": {
                        "velocityTarget": leftVelocityTarget,
                        "velocityMeasured": leftVelocityMeasured
                    },
                    "rightEncoder": {
                        "velocityTarget": rightVelocityTarget,
                        "velocityMeasured": rightVelocityMeasured
                    },
                    "heading": {
                        "velocityOffsetFromHeading": velocityOffsetFromHeading,
                        "headingTarget": headingTarget,
                        "currentHeading": currentHeading,
                    }
                })
            except ValueError:
                print("Failed to parse pid log", message)

    def handle_possible_navigate_log(message: str):
        # Expected format from firmware: "NAVIGATE" + <flag> + "," + <speed> + "," + <turn> + ";"
        # where <flag> may be 0/1, speed/turn are numeric.
        split = message.split(",")
        if len(split) != 3:
            return
        try:
            flag_raw = int(split[0].strip())
            # accept boolean-like values
            navigation_flag = False if flag_raw == 0 else True

            navigation_speed = float(split[1])
            navigation_turn = float(split[2])

            navigate_channel.log({
                "navigation_flag": navigation_flag,
                "navigation_speed": navigation_speed,
                "navigation_turn": navigation_turn,
            })
        except Exception:
            print("Failed to parse navigate log", message)

    read_and_log_serial()
