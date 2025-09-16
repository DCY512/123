#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import serial
import time
import threading
import re
import json
from std_msgs.msg import String
from collections import deque

# Hardware configuration
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Alibaba Cloud IoT configuration
WIFI_CONFIG = {
    "ssid": "DCY",
    "password": "12345678"
}

MQTT_CONFIG = {
    "client_id": r"iggisf93NrW.ROSPI|securemode=2\,signmethod=hmacsha256\,timestamp=1741498772243|",
    "username": "ROSPI&iggisf93NrW",
    "password": "84636b794ea1a4d3307ccab810685b76b806eb2c63f04f3333dd84de81146318",
    "broker": "iot-06z00emlemujeca.mqtt.iothub.aliyuncs.com",
    "port": 1883
}

class IoTManager:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self.connected = False
        self.last_sent = {
            'yaw': {'value': None, 'time': 0},
            'xy': {'value': (None, None), 'time': 0}
        }
        self.xy_queue = deque(maxlen=1)  # Thread-safe queue to store the latest XY data
        self.yaw_queue = deque(maxlen=1)  # Thread-safe queue to store the latest Yaw data
        self.forwarded_pub = rospy.Publisher("/forwarded_data", String, queue_size=10)  # Publisher for forwarded data

    def initialize_connection(self):
        if not self._init_serial():
            rospy.logerror("[INIT] Serial port initialization failed")
            return False
        
        command_sequence = [
            {
                "cmd": 'ATE0',
                "timeout": 3,
                "expect": "ATE0 OK",
                "desc": "Disable echo"
            },
            {
                "cmd": 'AT+CWMODE=1',
                "timeout": 5,
                "expect": "CWMODE OK",
                "desc": "Set STA mode"
            },
            {
                "cmd": r'AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com"',
                "timeout": 5,
                "expect": "CIPSNTPCFG OK",
                "desc": "Configure NTP server"
            },
            {
                "cmd": r'AT+CWJAP="%s","%s"' % (WIFI_CONFIG["ssid"], WIFI_CONFIG["password"]),
                "timeout": 15,
                "expect": "CWJAP:1,1",
                "desc": "Connect to WiFi"
            },
            {
                "cmd": r'AT+MQTTUSERCFG=0,1,"NULL","%s","%s",0,0,""' % 
                     (MQTT_CONFIG["username"], MQTT_CONFIG["password"]),
                "timeout": 10,
                "expect": "MQTTUSERCFG OK",
                "desc": "Configure MQTT user"
            },
            {
                "cmd": r'AT+MQTTCLIENTID=0,"%s"' % MQTT_CONFIG["client_id"],
                "timeout": 10,
                "expect": "MQTTCLIENTID OK",
                "desc": "Set ClientID"
            },
            {
                "cmd": r'AT+MQTTCONN=0,"%s",%d,1' % (MQTT_CONFIG["broker"], MQTT_CONFIG["port"]),
                "timeout": 20,
                "expect": "MQTTCONN OK",
                "desc": "Connect to MQTT broker"
            },
            {
                "cmd": r'AT+MQTTSUB=0,"/iggisf93NrW/ROSPI/user/get",1',
                "timeout": 10,
                "expect": "MQTTSUB OK",
                "desc": "Subscribe to topic"
            }
        ]
        
        for step in command_sequence:
            rospy.loginfo("[CMD] Executing: %s (%s)", step["desc"], step["cmd"])
            if not self._send_at_command(
                command=step["cmd"],
                expected="OK",
                timeout=step["timeout"],
                expect_desc=step["expect"]
            ):
                rospy.logerror("[FAIL] Command failed: %s", step["desc"])
                return False
            time.sleep(1)
        
        self.connected = True
        rospy.loginfo("[CONN] Connection established successfully")
        return True

    def _init_serial(self):
        try:
            with self.lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    port=SERIAL_PORT,
                    baudrate=BAUD_RATE,
                    timeout=1,
                    write_timeout=5,
                    rtscts=True
                )
                rospy.loginfo("[SERIAL] Serial port initialized %s@%d", SERIAL_PORT, BAUD_RATE)
                return True
        except Exception as e:
            rospy.logerr("[SERIAL] Serial init error: %s", str(e))
            return False

    def _send_at_command(self, command, expected="OK", timeout=5, retries=3, expect_desc=""):
        for attempt in range(retries):
            full_response = []
            try:
                with self.lock:
                    self.ser.flushInput()
                    self.ser.flushOutput()
                    self.ser.write(command + "\r\n")
                    rospy.loginfo("[TX] >> %s", command)

                start_time = time.time()
                success = False
                while time.time() - start_time < timeout:
                    with self.lock:
                        line = self.ser.readline().decode('utf-8', 'ignore').strip()
                        if line:
                            full_response.append(line)
                            rospy.loginfo("[RX] << %s", line)
                            
                            # Check if the line is an MQTT message
                            if line.startswith("+MQTTSUBRECV:"):
                                self.handle_mqtt_message(line)
                            
                            if expected in line:
                                success = True
                                break
                            if any(err in line for err in ["ERROR", "FAIL"]):
                                break
                    time.sleep(0.1)

                if success:
                    rospy.loginfo("[AT] Command success: %s", expect_desc)
                    return True
                
                rospy.logwarn("[RETRY] Attempt %d/%d: %s", 
                             attempt+1, retries, command)
                rospy.logdebug("[RESP] Full response:\n%s", "\n".join(full_response))
                time.sleep(1)
                
            except Exception as e:
                rospy.logerr("[COMM] Communication error: %s", str(e))
                time.sleep(2)
        
        rospy.logerror("[AT] Command failed: %s\nLast response: %s", 
                     command, "\n".join(full_response[-3:]))
        return False

    def publish_data(self, topic, value):
        try:
            num_value = float(value)
            if not (-1000.0 <= num_value <= 1000.0):
                rospy.logwarn("[DATA] Value out of range: %.2f", num_value)
                return False
        except (ValueError, TypeError):
            rospy.logerr("[DATA] Invalid value: %s", value)
            return False
        
        try:
            payload = json.dumps({"params": {topic: round(num_value, 3)}})
            escaped_payload = payload.replace('"', r'\"')
            cmd = 'AT+MQTTPUB=0,"/iggisf93NrW/ROSPI/user/update","{}",1,0'.format(escaped_payload)
            return self._send_at_command(
                cmd, 
                expected="OK", 
                timeout=3,
                expect_desc="Publish data to {}".format(topic)
            )
        except Exception as e:
            rospy.logerr("[JSON] Data construction failed: %s", str(e))
            return False

    def check_connection(self):
        return self._send_at_command(
            "AT+MQTTSTATUS?", 
            expected="STATUS:3", 
            expect_desc="Check MQTT connection status"
        )

    def handle_mqtt_message(self, message):
        """Handle incoming MQTT messages"""
        try:
            rospy.loginfo("[MQTT] Received message: %s", message)
            
            # Check if the message matches the expected format
            if message.startswith("+MQTTSUBRECV:"):
                # Extract the JSON payload from the message
                json_start = message.find("{")
                json_end = message.rfind("}") + 1  # Find the last closing brace
                if json_start != -1 and json_end != -1:
                    json_payload = message[json_start:json_end]
                    rospy.loginfo("[MQTT] Extracted JSON payload: %s", json_payload)
                    
                    try:
                        # Parse the JSON payload
                        data = json.loads(json_payload)
                        rospy.loginfo("[MQTT] Parsed JSON data: %s", data)
                        
                        # Check if the "params" field contains "HUO": 1
                        if "params" in data and isinstance(data["params"], dict):
                            if "HUO" in data["params"] and data["params"]["HUO"] == 1:
                                rospy.loginfo("[MQTT] Forwarding HUO:1 to /forwarded_data")
                                self.forwarded_pub.publish("HUO:1")  # Publish to ROS topic
                            else:
                                rospy.logwarn("[MQTT] HUO is not 1 or not found in params")
                        else:
                            rospy.logwarn("[MQTT] 'params' field is missing or not a dictionary")
                    except json.JSONDecodeError as e:
                        rospy.logerr("[MQTT] JSON decode error: %s", str(e))
                    except Exception as e:
                        rospy.logerr("[MQTT] Error parsing JSON: %s", str(e))
                else:
                    rospy.logwarn("[MQTT] No valid JSON payload found in message")
            else:
                rospy.logwarn("[MQTT] Message does not start with +MQTTSUBRECV:")
        except Exception as e:
            rospy.logerr("[MQTT] Message handling error: %s", str(e))

def handle_yaw(msg, manager):
    try:
        match = re.match(r"Yaw:\s*([+-]?\d+\.\d+)", msg.data)
        if not match:
            rospy.logwarn("[YAW] Invalid format: %s", msg.data)
            return
        
        current = float(match.group(1))
        # Store the latest Yaw data in the queue
        manager.yaw_queue.append(current)
    except Exception as e:
        rospy.logerr("[YAW] Processing error: %s", str(e))

def handle_xy(msg, manager):
    try:
        match = re.search(r"POSITION:\s*([+-]?\d+\.?\d*)[,\s]+([+-]?\d+\.?\d*)", msg.data)
        if not match:
            rospy.logwarn("[XY] Invalid format: %s (Expected: POSITION: x,y)", msg.data)
            return
        
        try:
            x = float(match.group(1))
            y = float(match.group(2))
        except ValueError:
            rospy.logerr("[XY] Coordinate conversion failed: %s", msg.data)
            return
        
        if not (-100.0 < x < 100.0 and -100.0 < y < 100.0):
            rospy.logwarn("[XY] Abnormal coordinate value: (%.2f, %.2f)", x, y)
            return
        
        # Store the latest XY data in the queue
        manager.xy_queue.append((x, y))
    except Exception as e:
        rospy.logerr("[XY] Processing error: %s", str(e))

def process_xy_data(manager):
    """Process the latest XY data every 0.2 seconds"""
    while not rospy.is_shutdown():
        try:
            if manager.xy_queue:
                x, y = manager.xy_queue[-1]  # Get the latest XY data
                last = manager.last_sent['xy']
                now = time.time()
                
                if (now - last['time'] >= 0.2):  # Process every 0.2 seconds
                    success_x = manager.publish_data("DCYX", x)
                    success_y = manager.publish_data("DCYY", y)
                    if success_x and success_y:
                        manager.last_sent['xy'] = {'value': (x, y), 'time': now}
                        rospy.loginfo("[XY] Published: (%.2f, %.2f)", x, y)
            time.sleep(0.2)  # Sleep for 0.2 seconds
        except Exception as e:
            rospy.logerr("[XY] Processing error: %s", str(e))
            time.sleep(0.2)

def process_yaw_data(manager):
    """Process the latest Yaw data every 0.2 seconds"""
    while not rospy.is_shutdown():
        try:
            if manager.yaw_queue:
                yaw = manager.yaw_queue[-1]  # Get the latest Yaw data
                last = manager.last_sent['yaw']
                now = time.time()
                
                if (now - last['time'] >= 0.2):  # Process every 0.2 seconds
                    if manager.publish_data("Angle", yaw):
                        manager.last_sent['yaw'] = {'value': yaw, 'time': now}
                        rospy.loginfo("[YAW] Published: %.2f¡ã", yaw)
            time.sleep(0.2)  # Sleep for 0.2 seconds
        except Exception as e:
            rospy.logerr("[YAW] Processing error: %s", str(e))
            time.sleep(0.2)

def connection_watchdog(manager):
    while not rospy.is_shutdown():
        try:
            if not manager.check_connection():
                rospy.logwarn("[WATCHDOG] Connection lost, reconnecting...")
                manager.initialize_connection()
            time.sleep(10)
        except Exception as e:
            rospy.logerr("[WATCHDOG] Watchdog error: %s", str(e))
            time.sleep(5)

def main():
    rospy.init_node("iot_gateway")
    
    manager = IoTManager()
    
    rospy.loginfo("[MAIN] Starting connection sequence...")
    while not rospy.is_shutdown():
        if manager.initialize_connection():
            break
        rospy.logerr("[MAIN] Connection failed, retrying in 5 seconds...")
        time.sleep(5)
    
    watchdog = threading.Thread(target=connection_watchdog, args=(manager,))
    watchdog.daemon = True
    watchdog.start()
    
    # Start a thread to process XY data every 0.2 seconds
    xy_processor = threading.Thread(target=process_xy_data, args=(manager,))
    xy_processor.daemon = True
    xy_processor.start()
    
    # Start a thread to process Yaw data every 0.2 seconds
    yaw_processor = threading.Thread(target=process_yaw_data, args=(manager,))
    yaw_processor.daemon = True
    yaw_processor.start()
    
    rospy.Subscriber("/filtered_yaw", String, lambda msg: handle_yaw(msg, manager))
    rospy.Subscriber("/forwarded_data", String, lambda msg: handle_xy(msg, manager))
    
    rospy.loginfo("[MAIN] IoT Gateway Ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
