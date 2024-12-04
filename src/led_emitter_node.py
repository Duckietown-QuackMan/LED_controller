#!/usr/bin/env python3

import time
from typing import List

import yaml
import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA, Int32, Bool




class LEDController:
    def __init__(self):
        # initialise the ROS node
        node_name = "led_emitter_node"
        rospy.init_node(node_name)
        print("LED emitter node initialised")


        self.setup_params()
        self.setup_publishers_and_subscribers()
        self.led_pattern_on_start()




    def setup_params(self) -> None:
        """
        Setup the parameters for the node reading them from the ROS parameter server.
        - self.name_sub_game_state: name of the game state channel
        - self.name_sub_ghost_bot: name of the channel informing if a GhostBot is seen
        - self.name_sub_ghost_bot_b: name of the channel informing if the back a GhostBot is seen
        - self.name_sub_quack_man: name of the channel informing if the QuackMan is seen
        - self.name_sub_x_sec: name of the channel for detecting x-sec
        - self.name_pub_lane_following: if set to true the lane following node should start driving
        - self.name_pub_x_sec_go: if set set to true the x-sec navigation should start navigating through the cross section
        - self.name_pub_game_over: if set to true the QuackMan was detected and the game is over
        """

        def get_rosparam(name):
            if rospy.has_param(name):
                param = rospy.get_param(name)
            else:
                txt_error = f"Parameter '{name}' is not set."
                rospy.logerr(txt_error)
                raise KeyError(txt_error)
            return param
        
        # variable params
        # self.vehicle_name = rospy.get_param("~vehicle_name")

        self.vehicle_name = get_rosparam("~vehicle_name")
        
        # self.frequency = get_rosparam("~colours/frequency")
        # self.intensity = get_rosparam("~colours/intensity")

        # Load colors from 'colours/COLORS' namespace
        # colours = rospy.get_param("~colours/colours")
        # self.colour_objects = {name: ColorRGBA(**values) for name, values in colours.items()}
        # Load colors from 'colours' namespace
        # colours = get_rosparam("~colours")
        # self.colour_objects = {name: ColorRGBA(**values) for name, values in colours.items()}

        # Load the colours.yaml file manually
        colours_yaml_path = get_rosparam("~colours_param_file_name")
        rospy.loginfo(f"Loading colours from {colours_yaml_path}")
        with open(colours_yaml_path, 'r') as file:
            colours = yaml.safe_load(file)["colours"]

        # Convert colors to ColorRGBA objects
        self.colour_objects = {name: ColorRGBA(**values) for name, values in colours.items()}
        rospy.loginfo(f"Loaded colours: {self.colour_objects}")

        # Debugging: Log the loaded colors
        rospy.loginfo(f"Loaded colors: {self.colour_objects}")

        self.score = None
        # # tag info
        # yaml_path = rospy.get_param("~tag_params_path", "tag_config.yaml")
        # with open(yaml_path, 'r') as f:
        #     self.tag_config = yaml.safe_load(f)

        # Load topics from 'topics' namespace
       

        # topics params
        self.name_sub_all_checkpoints_collected = get_rosparam("~topics/sub/all_checkpoints_collected")
        self.name_sub_score_update = get_rosparam("~topics/sub/score_update")
        self.name_sub_checkpoint_timeout = get_rosparam("~topics/sub/checkpoint_timeout")
        self.name_pub_quackman_found = get_rosparam("~topics/sub/quack_man")

        self.name_pub_led_pattern = self.vehicle_name + get_rosparam("~topics/pub/led_pattern")


        rospy.loginfo(f"Resolved topics name loading")


        
        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """
        self.sub_score_update = rospy.Subscriber(self.name_sub_score_update,       
                                          Int32, 
                                          self.cb_score_update, 
                                          queue_size=10)
        self.sub_all_checkpoints_collected = rospy.Subscriber(self.name_sub_all_checkpoints_collected,       
                                          Bool, 
                                          self.cb_all_cp_collected, 
                                          queue_size=10)
        self.sub_checkpoint_timeout = rospy.Subscriber(self.name_sub_checkpoint_timeout,
                                            Bool,
                                            self.cb_cp_timeout,
                                            queue_size=10)
        self.sub_quackman_found = rospy.Subscriber(self.name_pub_quackman_found,
                                            Bool,
                                            self.cb_quackman_found,
                                            queue_size=10)

        self.pub_led_pattern = rospy.Publisher(self.name_pub_led_pattern, 
                                             LEDPattern, 
                                             queue_size=10)
        


    def cb_score_update(self, msg):
        rospy.loginfo(f"score: {msg.data}")
        # Ensure score is within range
        self.score = msg.data
        if self.score <= 0 or self.score > 15:
            rospy.logwarn("Score out of range (1-15). Ignoring.")
            return
        
        # Convert score to binary and map to LED states
        binary = f"{self.score:04b}"  # Convert to 4-bit binary string
        rospy.loginfo(f"Binary representation: {binary}")
        # Add a zero in the middle
        binary_with_zero = binary[:2] + '0' + binary[2:]
        rospy.loginfo(f"Binary representation: {binary_with_zero}")


        # Create LED pattern
        led_msg = LEDPattern()
        led_msg.rgb_vals = []
        led_msg.color_mask = []
        for i, bit in enumerate(binary_with_zero):
            print(i, bit)
            if bit == '1':
                # led_msg.rgb_vals.append(self.colour_objects['COLOUR_WHITE'])  # ON (White)
                led_msg.rgb_vals.append(ColorRGBA(1.0, 1.0, 1.0, 1.0))  # ON (White)
                led_msg.color_mask.append(1)
            else:
                led_msg.rgb_vals.append(self.colour_objects['COLOUR_OFF'])  # OFF
                led_msg.color_mask.append(0)

        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        rospy.loginfo("SCORE-LED published.")


    def cb_all_cp_collected(self, msg):
        if msg.data == False:
            rospy.loginfo("All checkpoints not collected. Ignoring.")
            return
        # Create LED pattern
        led_msg = LEDPattern()
        led_msg.rgb_vals = []
        led_msg.color_mask = []
        for i in range(5):
            led_msg.rgb_vals.append(self.colour_objects['COLOUR_GREEN'])  
            led_msg.color_mask.append(1)
        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        rospy.loginfo("ALL_CP_COLLECTED-LED published.")
    
    def cb_cp_timeout(self, msg):
        if msg.data == False:
            rospy.loginfo("All checkpoints not collected. Ignoring.")
            return
        # Create LED pattern
        led_msg = LEDPattern()
        led_msg.rgb_vals = []
        led_msg.color_mask = []
        for i in range(5):
            led_msg.rgb_vals.append(self.colour_objects['COLOUR_RED'])  
            led_msg.color_mask.append(1)
        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        rospy.loginfo("CP_TIMEOUT-LED published.")

    def cb_quackman_found(self, msg):
        if msg.data == False:
            rospy.loginfo("All checkpoints not collected. Ignoring.")
            return
        # Create LED pattern
        led_msg = LEDPattern()
        led_msg.rgb_vals = []
        led_msg.color_mask = []
        for i in range(5):
            led_msg.rgb_vals.append(self.colour_objects['COLOUR_RED'])  
            led_msg.color_mask.append(1)
        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        rospy.loginfo("QM_found-LED published.")


    def led_pattern_on_start(self):
        # Create LED pattern
        led_msg = LEDPattern()
        led_msg.rgb_vals = []
        led_msg.color_mask = []
        for i in range(5):
            led_msg.rgb_vals.append(self.colour_objects['COLOUR_BLUE'])  
            led_msg.color_mask.append(1)
        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink
        rospy.loginfo(led_msg)

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        rospy.loginfo("WAITING FOR FIRST CP-LED published.")
        pass
        

if __name__ == "__main__":
    LEDController()
    
    rospy.spin()



# def blinker():
#     robot_name: str = get_robot_name()
#     # initialize node
#     rospy.init_node('blinker', anonymous=True)
#     # setup publisher
#     publisher = rospy.Publisher(
#         f"/{robot_name}/led_driver_node/led_pattern",
#         LEDPattern,
#         queue_size=1,
#         tcp_nodelay=True,
#     )
#     # back to default when shutting down
#     rospy.on_shutdown(lambda: regular_pattern(publisher))
#     # blink
#     dt: float = 1.0 / FREQUENCY
#     bit: int = 0
#     while not rospy.is_shutdown():
#         intensity: float = bit * INTENSITY
#         publisher.publish(
#             LEDPattern(
#                 rgb_vals=[ColorRGBA(*RGB_AMBER, intensity)] * 5
#             )
#         )
#         time.sleep(dt)
#         bit = 1 - bit
