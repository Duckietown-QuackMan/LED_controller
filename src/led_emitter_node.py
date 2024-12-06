#!/usr/bin/env python3
import time
from typing import List

import yaml
import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA, Int32, Bool, String



class LEDController:
    def __init__(self):
        # initialise the ROS node
        node_name = "led_emitter_node"
        rospy.init_node(node_name)
        print("LED emitter node initialised")


        self.setup_params()
        self.setup_publishers_and_subscribers()
        self.game_running = False




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
        self.vehicle_name = get_rosparam("~vehicle_name")
        

        # Load the colours.yaml file manually
        colours_yaml_path = get_rosparam("~colours_param_file_name")
        rospy.loginfo(f"Loading colours from {colours_yaml_path}")
        with open(colours_yaml_path, 'r') as file:
            colours = yaml.safe_load(file)["colours"]

        # Convert colors to ColorRGBA objects
        self.colour_objects = {name: ColorRGBA(**values) for name, values in colours.items()}
        rospy.loginfo(f"Loaded colours: {self.colour_objects}")

        self.score = None

        # topics params
        self.name_sub_all_checkpoints_collected = get_rosparam("~topics/sub/all_checkpoints_collected")
        self.name_sub_score_update = get_rosparam("~topics/sub/score_update")
        self.name_sub_checkpoint_timeout = get_rosparam("~topics/sub/checkpoint_timeout")
        self.name_pub_quackman_found = get_rosparam("~topics/sub/quack_man")
        self.name_sub_game_state = get_rosparam("~topics/sub/game_state")

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
        self.sub_game_State = rospy.Subscriber(self.name_sub_game_state,
                                            String,
                                            self.cb_game_state,
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
                led_msg.rgb_vals.append(self.colour_objects['COLOUR_WHITE'])  # ON (White)
                # led_msg.rgb_vals.append(ColorRGBA(1.0, 1.0, 1.0, 1.0))  # ON (White)
                led_msg.color_mask.append(1)
            else:
                led_msg.rgb_vals.append(self.colour_objects['COLOUR_OFF'])  # OFF
                led_msg.color_mask.append(0)

        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        rospy.loginfo("LED_pattern-SCORE set.")


    def cb_all_cp_collected(self, msg):
        if msg.data == False:
            rospy.loginfo("All checkpoints not collected yet. Ignoring.")
            return
        self.set_led_pattern("COLOUR_GREEN")
        rospy.loginfo("LED_pattern-ALL_CP_COLLECTED set.")
    
    def cb_cp_timeout(self, msg):
        if msg.data == False:
            rospy.loginfo("Checkpoint timeout not reached. Ignoring.")
            return
        self.set_led_pattern("COLOUR_RED")
        rospy.loginfo("LED_pattern-CP_TIMEOUT set.")

    def cb_quackman_found(self, msg):
        if msg.data == False:
            rospy.loginfo("Quackman not found. Ignoring.")
            return
        self.set_led_pattern("COLOUR_RED")
        rospy.loginfo("LED_pattern-QM_found set.")
    
    def cb_game_state(self, msg):
        print(msg.data, self.game_running)
        if msg.data == "IDLE":
            # indicate game is idle, waiting for all bots to connect
            self.set_led_pattern("COLOUR_BLUE")
        elif msg.data == "RUNNING" and not self.game_running:
            # indicates game has started, but no checkpoints detected yet
            self.game_running = True
            self.set_led_pattern("COLOUR_OFF")

    def set_led_pattern(self, colour_name: str):
        # Create LED pattern
        led_msg = LEDPattern()
        led_msg.rgb_vals = []
        led_msg.color_mask = []
        for i in range(5):
            led_msg.rgb_vals.append(self.colour_objects[colour_name])  
            led_msg.color_mask.append(1)
        led_msg.frequency = 0.0  # No blinking
        led_msg.frequency_mask = [0, 0, 0, 0]  # No LEDs blink
        rospy.loginfo(led_msg)

        # Publish LED pattern
        self.pub_led_pattern.publish(led_msg)
        pass

        

if __name__ == "__main__":
    LEDController()
    rospy.spin()