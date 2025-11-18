import serial
import datetime
import foxglove
from foxglove import Channel
from foxglove.channels import Vector2Channel, LogChannel
from foxglove.schemas import Timestamp, Vector2, Log

SERIAL_PORT = "/dev/cu.usbmodem01"


file_name = f"recordings/recording_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.mcap"

with foxglove.open_mcap(file_name) as writer:
    print("Recording at:", file_name)
    server = foxglove.start_server()

    log_channel = LogChannel("/esp_logs")
    pid_data_channel = Channel("/prev_pid", message_encoding="json")

    dprg_nav_current_pos_channel = Vector2Channel("/json_pid_current_position")
    dprg_nav_target_pos_channel = Vector2Channel("/json_pid_target_position")

    esp = serial.Serial(port=SERIAL_PORT, baudrate=115200, timeout=.1) 

    def read_and_log_serial(ser: serial.Serial):
        while True:
            try:
                line = ser.readline()
                if line:
                    message = line.decode(errors="replace").strip()
                    if message.endswith("**"):
                        handle_possible_pid_log(message[:-2])
                    elif message.startswith("||") and message.endswith(";"):
                        handle_possible_dprg_nav_log(message[2:-1])
                    else:
                        log_channel.log(Log(message=message, timestamp=Timestamp.now()))
            except Exception as e:
                print({"message": f"Error reading serial: {str(e)}"})

    def handle_possible_dprg_nav_log(message: str):
        split = message.split(",")
        if len(split) == 4:
            try:
                parsed = [float(s) for s in split]
                X, X_target, Y, Y_target, theta = parsed
                # TODO: send theta to foxglove
                dprg_nav_current_pos_channel.log(Vector2(x=X, y=Y))
                dprg_nav_target_pos_channel.log(Vector2(x=X_target, y=Y_target))
            except ValueError:
                print("tuff")

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
                print("tuff")

    read_and_log_serial(esp)
