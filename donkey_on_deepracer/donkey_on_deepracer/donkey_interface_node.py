"""
Haoru Xue | July 2021 | hxue@ucsd.edu
"""

import rclpy
from rclpy.node import Node

from deepracer_interfaces_pkg.msg import CameraMsg, ServoCtrlMsg
from deepracer_interfaces_pkg.srv import VideoStateSrv, ServoGPIOSrv

import json, threading, time, cv_bridge, cv2, socket, base64

IMAGE_MSG_TOPIC = "/camera_pkg/video_mjpeg"
SERVO_TOPIC = "/ctrl_pkg/servo_msg"
HOST, PORT = "", 9091
PERIOD_INBOUND = 0.015
PERIOD_OUTBOUND = 0.015

class DonkeyInterfaceNode(Node):
    def __init__(self):
        super().__init__("donkey_interface_node")
        self.control_pub_ = self.create_publisher(ServoCtrlMsg, SERVO_TOPIC, 2)
        self.bridge_ = cv_bridge.CvBridge()
        self.server_ = DonkeyServer(self)
        self.image_sub_ = self.create_subscription(CameraMsg, IMAGE_MSG_TOPIC, self.image_callback, 2)
        self.toggle_camera(1)
        self.toggle_gpio(1)
    
    def image_callback(self, image_msg:CameraMsg):
        if len(image_msg.images) > 0:
            image_arr = self.bridge_.imgmsg_to_cv2(image_msg.images[-1])
            self.server_.send_image(image_arr)
        else:
            self.get_logger().warning("No new image in the sequence.")
    
    def destroy_node(self) -> bool:
        self.server_.shutdown()     
        return super().destroy_node()

    def toggle_camera(self, enable:int=0):
        """ Let the camera node start (stop) streaming """
        action = "" if enable else "de"
        cli = self.create_client(VideoStateSrv, "/camera_pkg/media_state")

        while not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for camera node to launch...')

        req = VideoStateSrv.Request()
        req.activate_video = enable
        self.get_logger().info(f"{action}activate camera")
        cli.call_async(req)
        #if error_code:
        #    self.get_logger().error(f"Camera node is unable to {action}activate camera.")

    def toggle_gpio(self, enable:int=0):
        """ 
        Let the servo node toggle GPIO 
        """
        action = "enable" if enable else "disable"
        enable = 1 if enable == 0 else 0 # The actual enabling signal is flipped
        cli = self.create_client(ServoGPIOSrv, "/servo_pkg/servo_gpio")

        while not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for servo node to launch...')

        req = ServoGPIOSrv.Request()
        req.enable = enable
        self.get_logger().info(f"{action} GPIO")
        cli.call_async(req)

        #if error_code:
        #    self.get_logger().error(f"Servo node is unable to {action} GPIO.")

class DonkeyServer:
    def __init__(self, node:DonkeyInterfaceNode) -> None:
        self.on = True
        self.server_active = False
        self.outbound_buffer_lock_ = threading.Lock()
        self.outbound_buffer_ = bytes()
        self.inbound_buffer = bytes()

        self.img_w = 160
        self.img_h = 120

        self.node_ = node
        self.t = threading.Thread(target=self.server_thread, daemon=True)
        self.t.start()

    def server_thread(self):
        self.publish_control()
        self.serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serv.bind((HOST, PORT))
        self.serv.listen(1)
        while self.on:
            self.node_.get_logger().info(f"Awaiting DonkeyCar connection on port {PORT}...")
            self.conn, addr = self.serv.accept()
            self.server_active = True
            self.node_.get_logger().info(f"DonkeyCar connected. IP: {addr}.")
            # time.sleep(3.0)
            self.addToOutbound("{\"msg_type\": \"need_car_config\"}\n")
            self.publish_control()
            try:
                self.t_in = threading.Thread(target=self.handle_inbound, daemon=True)
                self.t_out = threading.Thread(target=self.handle_outbound, daemon=True)
                self.t_in.start()
                self.t_out.start()
                while self.on and self.server_active:
                    time.sleep(1)
            except Exception as e:
                self.node_.get_logger().error(str(e))
                self.publish_control()
                continue
            finally:
                self.node_.get_logger().warning("DonkeyCar disconnected. Vehicle stopped.")
                self.conn.close()
                self.server_active = False
                self.publish_control()
        self.publish_control()

    def handle_inbound(self):
        try:
            while self.on and self.server_active:
                    data = self.conn.recv(1024)
                    self.inbound_buffer += data
                    while self.inbound_buffer: # are there leftover chars in the buffer?
                        termination = self.inbound_buffer.find("}".encode("utf-8")) # search the buffer for packet ending
                        if termination >= 0: # if packet is complete
                            inbound_msg = str(self.inbound_buffer[0:termination+1], "utf-8")
                            # self.node_.get_logger().debug(f"Inbound message: {inbound_msg}")
                            self.on_msg_recv(inbound_msg)
                            self.inbound_buffer = self.inbound_buffer[termination+1:] # remove parsed packet from buffer
                        else: # incomplete packet. wait for the next receiving.
                            break
                    time.sleep(PERIOD_INBOUND)
        except (BrokenPipeError, ConnectionResetError) as e:
            self.server_active = False
            return

    def handle_outbound(self):
        try:
            while self.on and self.server_active:
                self.outbound_buffer_lock_.acquire()
                if self.outbound_buffer_:
                    to_send = bytes(self.outbound_buffer_)
                    self.outbound_buffer_ = bytes()
                    self.outbound_buffer_lock_.release()
                    # self.node_.get_logger().debug(f"Outbound message: {self.outbound_buffer_}")
                    self.conn.sendall(to_send)
                else:
                    self.outbound_buffer_lock_.release()
            time.sleep(PERIOD_OUTBOUND)
        except (BrokenPipeError, ConnectionResetError) as e:
            self.server_active = False
            return

    def on_msg_recv(self, msg:str):
        try:
            msg_json = json.loads(msg)
            if msg_json["msg_type"] == "control":
                self.publish_control(float(msg_json.get("steering", 0.0)), float(msg_json.get("throttle", 0.0)))
            elif msg_json["msg_type"] == "cam_config":
                self.node_.get_logger().info(f"Inbound message: {msg}")
                self.configure_camera(msg_json)
            elif msg_json["msg_type"] == "get_protocol_version":
                self.node_.get_logger().info(f"Inbound message: {msg}")
                self.addToOutbound("{\"msg_type\": \"protocol_version\",\"version\": \"2\"}\n")
            elif msg_json["msg_type"] == "load_scene":
                self.node_.get_logger().info(f"Inbound message: {msg}")
                self.addToOutbound("{\"msg_type\": \"scene_loaded\"}\n")
                self.addToOutbound("{\"msg_type\": \"need_car_config\"}\n")
                self.addToOutbound("{\"msg_type\": \"car_loaded\"}\n")
            elif msg_json["msg_type"] == "get_scene_names":
                self.node_.get_logger().info(f"Inbound message: {msg}")
                self.addToOutbound("{\"msg_type\" : \"scene_names\", \"scene_names\" : [ \"generated_road\", \"warehouse\", \"sparkfun_avc\", \"generated_track\" ]}\n")
            else:
                self.node_.get_logger().info(f"Inbound message: {msg}")
        except Exception as e:
            self.node_.get_logger().error(str(e))
            self.node_.get_logger().error(f"Failed to parse incoming message: {msg}")
            self.publish_control()

    def send_image(self, image):
        resized_image = cv2.resize(image, (self.img_w, self.img_h))
        _, img_buffer = cv2.imencode(".jpg", resized_image)
        img_b64 = str(base64.b64encode(img_buffer), "utf-8")
        outbound_dict = { "msg_type" : "telemetry", 
                            "steering_angle" : 0.0, 
                            "throttle" : 0.0, 
                            "speed" : 1.0, 
                            "image" : img_b64, 
                            "hit" : "None", 
                            "pos_x" : 0.0, 
                            "pos_y" : 0.0, 
                            "pos_z" : 0.0, 
                            "accel_x" : 0.0, 
                            "accel_y" : 0.0, 
                            "accel_z" : 0.0, 
                            "gyro_x" : 0.0, 
                            "gyro_y" : 0.0, 
                            "gyro_z" : 0.0, 
                            "gyro_w" : 0.0,
                            "pitch" : 0.0, 
                            "roll" : 0.0, 
                            "yaw" : 0.0,
                            "activeNode" : 0,
                            "totalNodes" : 0,
                            "cte" : 0.0
                        }
        self.addToOutbound(json.dumps(outbound_dict) + "\n")

    def publish_control(self, steering=0.0, throttle=0.0):
        ctrl_msg = ServoCtrlMsg()
        ctrl_msg.angle = steering
        ctrl_msg.throttle = throttle
        self.node_.control_pub_.publish(ctrl_msg)

    def shutdown(self):
        self.node_.get_logger().info("Shutting down DonkeyCar connection...")
        self.on = False
        self.t.join()
        self.publish_control()
        self.serv.close()

    def configure_camera(self, config_dict:dict):
        img_w = int(config_dict.get("img_w", 160))
        img_h = int(config_dict.get("img_h", 120))
        if img_w > 0 and img_h > 0:
            self.img_w, self.img_h = img_w, img_h
            self.node_.get_logger().info(f"Image configuration received. Width: {self.img_w}, Height: {self.img_h}.")

    def addToOutbound(self, msg: str):
        self.outbound_buffer_lock_.acquire()
        self.outbound_buffer_ += msg.encode("utf-8")
        self.outbound_buffer_lock_.release()

def main(args=None):
    rclpy.init(args=args)
    node = DonkeyInterfaceNode()
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()