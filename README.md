# LED_controller
This package controls the LED of the duckies to give some visual feedback to the player during the game.

## led_emitter_node
The LEDController node controls the LED patterns and colors based on the game state and events. Below is a description of when the LEDs show specific colors and the pattern used to display the score.

#### LED Colors
1. **Game State:**
    - **IDLE**: LEDs show blue.
    - **RUNNING**: LEDs show yellow.
    - **GAME_OVER**: LEDs show red.
2. **All Checkpoints Collected:**
    - LEDs show green.
3. **Checkpoint Timeout:**
    - LEDs show red.
4. **QuackMan Found:**
    - LEDs show red.

**Score Pattern**
When the score is updated, the LEDs display a pattern based on the binary representation of the score:

### Subscribed Topics

The LEDController node listens to the following topics to control the LED patterns and colors:

1. **Game State:**
   - **Topic:** `/quack_man/game_state`
   - **Message Type:** std_msgs/String
   - **Description:** Receives the current game state. The game state can be `IDLE`, `RUNNING`, or `GAME_OVER`, which sets the LEDs to blue, yellow, or red, respectively.

2. **Score Update:**
   - **Topic:** `/quack_man/score_update`
   - **Message Type:** std_msgs/Int32
   - **Description:** Receives the current score. The score is displayed on the LEDs in a binary pattern.

3. **All Checkpoints Collected:**
   - **Topic:** `/quack_man/all_checkpoints_collected`
   - **Message Type:** std_msgs/Bool
   - **Description:** Indicates whether all checkpoints have been collected. If true, the LEDs are set to green.

4. **Checkpoint Timeout:**
   - **Topic:** `/quack_man/checkpoint_timeout`
   - **Message Type:** std_msgs/Bool
   - **Description:** Indicates whether a checkpoint timeout has occurred. If true, the LEDs are set to red.

5. **QuackMan Found:**
   - **Topic:** `/quack_man/quack_man`
   - **Message Type:** std_msgs/Bool
   - **Description:** Indicates whether QuackMan has been found. If true, the LEDs are set to red.
