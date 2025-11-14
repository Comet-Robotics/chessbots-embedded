import json
import foxglove
from foxglove import Channel
from foxglove.channels import Vector2Channel, LogChannel
from foxglove.schemas import Timestamp, Vector2, Log
import serial
import threading
import datetime

SERIAL_PORT = "/dev/cu.usbmodem01"


file_name = f"recordings/recording_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.mcap"
writer = foxglove.open_mcap(file_name)
server = foxglove.start_server()


log_channel = LogChannel("/esp_logs")
prev_pid_data_channel = Channel("/prev_pid", message_encoding="json")
json_pid_data_channel = Channel("/json_pid", message_encoding="json")


json_pid_current_pos_channel = Vector2Channel("/json_pid_current_position")
json_pid_target_pos_channel = Vector2Channel("/json_pid_target_position")


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
                    handle_possible_json_pid_log(message[2:-1])
                else:
                    log_channel.log(Log(message=message, timestamp=Timestamp.now()))
        except Exception as e:
            print({"message": f"Error reading serial: {str(e)}"})

def handle_possible_json_pid_log(message: str):
    split = message.split(",")
    if len(split) == 4:
        try:
            parsed = [float(s) for s in split]
            X, X_target, Y, Y_target = parsed
            json_pid_current_pos_channel.log(Vector2(x=X, y=Y))
            json_pid_target_pos_channel.log(Vector2(x=X_target, y=Y_target))
        except:
            print("tuff")

def handle_possible_pid_log(message: str):
    split = message.split(",")
    if len(split) == 9:
        try:
            parsed = [float(s) for s in split]
            leftVelocityTarget, leftVelocityMeasured, _, rightVelocityTarget, rightVelocityMeasured, _, velocityOffsetFromHeading, headingTarget, currentHeading = parsed
            prev_pid_data_channel.log({
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
        except:
            print("tuff")


# # Run the serial reading in a background thread to avoid blocking main thread
# serial_thread = threading.Thread(target=read_and_log_serial, daemon=True)
# serial_thread.start()
# serial_thread.join()

read_and_log_serial(esp)
