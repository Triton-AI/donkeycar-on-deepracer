"""
Haoru Xue | July 2021 | hxue@ucsd.edu
"""

import rclpy
from rclpy.node import Node

from deepracer_interfaces_pkg.msg import CameraMsg, ServoCtrlMsg

import json, threading, time, cv_bridge, cv2, socket, base64, select

IMAGE_MSG_TOPIC = "/camera_pkg/video_mjpeg"
SERVO_TOPIC = "/ctrl_pkg/servo_msg"
HOST, PORT = "", 9091

class DonkeyInterfaceNode(Node):
    def __init__(self):
        super().__init__("donkey_interface_node")
        self.control_pub_ = self.create_publisher(ServoCtrlMsg, SERVO_TOPIC, 2)
        self.bridge_ = cv_bridge.CvBridge()
        self.server_ = DonkeyServer(self)
        self.image_sub_ = self.create_subscription(CameraMsg, IMAGE_MSG_TOPIC, self.image_callback, 2)
    
    def image_callback(self, image_msg:CameraMsg):
        if len(image_msg.images) > 0:
            image_arr = self.bridge_.imgmsg_to_cv2(image_msg.images[-1])
            self.server_.send_image(image_arr)
    
    def destroy_node(self) -> bool:
        self.server_.shutdown()     
        return super().destroy_node()


class DonkeyServer:
    def __init__(self, node:DonkeyInterfaceNode) -> None:
        self.on = True
        self.outbound_buffer_lock_ = threading.Lock()
        self.outbound_buffer_ = None
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
            self.node_.get_logger().info(f"DonkeyCar connected. IP: {addr}.")
            self.publish_control()
            try:
                timer_period_in = 0.01
                timer_period_out = 0.05
                self.timer_in = self.node_.create_timer(timer_period_in, self.handle_inbound)
                self.timer_out = self.node_.create_timer(timer_period_out, self.handle_outbound)
                while True:
                    time.sleep(1)
            except Exception as e:
                self.node_.get_logger().error(str(e))
                self.publish_control()
                self.node_.get_logger().warning("DonkeyCar disconnected. Vehicle stopped.")
                continue
            finally:
                self.conn.close()
                self.timer_in.cancel()
                self.timer_out.cancel()
        self.publish_control()

    def handle_inbound(self):
        self.inbound_buffer += self.conn.recv(1024)
        while self.inbound_buffer: # are there leftover chars in the buffer?
            termination = self.inbound_buffer.find("}".encode("utf-8")) # search the buffer for packet ending
            if termination >= 0: # if packet is complete
                inbound_msg = str(self.inbound_buffer[0:termination+1], "utf-8")
                self.node_.get_logger().debug(f"Inbound message: {inbound_msg}")
                self.on_msg_recv(inbound_msg)
                self.inbound_buffer = self.inbound_buffer[termination+1:] # remove parsed packet from buffer
            else: # incomplete packet. wait for the next receiving.
                break

    def handle_outbound(self):
        self.outbound_buffer_lock_.acquire()
        if self.outbound_buffer_ is not None:
            to_send = bytes(self.outbound_buffer_)
            self.outbound_buffer_ = None
            self.outbound_buffer_lock_.release()
            self.node_.get_logger().debug(f"Outbound message: {self.outbound_buffer_}")
            self.conn.sendall(to_send)
        else:
            self.outbound_buffer_lock_.release()

    def on_msg_recv(self, msg:str):
        try:
            msg_json = json.loads(msg)
            if msg_json["msg_type"] == "control":
                self.publish_control(float(msg_json.get("steering", 0.0)), float(msg_json.get("throttle", 0.0)))
            elif msg_json["msg_type"] == "cam_config":
                self.configure_camera(msg_json)
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
                            "steering_angle" : "0.0", 
                            "throttle" : "0.0", 
                            "speed" : "1.0", 
                            "image" : img_b64, 
                            "hit" : "None", 
                            "pos_x" : "0.0", 
                            "pos_y" : "0.0", 
                            "pos_z" : "0.0", 
                            "accel_x" : "0.0", 
                            "accel_y" : "0.0", 
                            "accel_z" : "0.0", 
                            "gyro_x" : "0.0", 
                            "gyro_y" : "0.0", 
                            "gyro_z" : "0.0", 
                            "gyro_w" : "0.0",
                            "pitch" : "0.0", 
                            "roll" : "0.0", 
                            "yaw" : "0.0",
                            "activeNode" : "0",
                            "totalNodes" : "0",
                            "cte" : "0"
                        }
        self.outbound_buffer_lock_.acquire()
        self.outbound_buffer_ = bytes(json.dumps(outbound_dict) + "\n", "utf-8")
        self.outbound_buffer_lock_.release()
        self.node_.get_logger().info("Image ready to send.")

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

    def configure_camera(self, config_dict:dict):
        self.img_w = int(config_dict.get("img_w", 160))
        self.img_h = int(config_dict.get("img_h", 120))
        self.node_.get_logger().info(f"Image configuration received. Width: {self.img_w}, Height: {self.img_h}.")



def main(args=None):
    rclpy.init(args=args)
    node = DonkeyInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()