# Electrical / Controls Description
# REAL-BOT


## Motors

| Subsystem         | Controller Type | Motor Type        | CAN ID | Position    | PDH ID |
| ----------------- | --------------- | ----------------- | ------ | ----------- | ------ |
| Drive             |                 |                   |        |             |        |
|   (MO) Drive      | Rev Spark Max   | NEO Brushless     | 11     | Left Front  |  9     |
|   (M0) Turn       | Rev Spark Flex  | NEO Brushless 550 | 21     | Left Front  |  8     |
|   (M1) Drive      | Rev Spark Max   | NEO Brushless     | 12     | Right Front | 10     |
|   (M1) Turn       | Rev Spark Flex  | NEO Brushless 550 | 22     | Right Front | 11     |
|   (M2) Drive      | Rev Spark Max   | NEO Brushless     | 13     | Left Rear   | 13     |
|   (M2) Turn       | Rev Spark Flex  | NEO Brushless 550 | 23     | Left Rear   | 12     |
|   (M3) Drive      | Rev Spark Max   | NEO Brushless     | 14     | Right Rear  | 19     |
|   (M3) Turn       | Rev Spark Flex  | NEO Brushless 550 | 24     | Right Rear  | 18     |
| Shooter (Leader)  | Kraken x44      | Brushless DC(BLDC)| 30     | --          | 17     |
| Shooter(Follower) | Kraken x44      | Brushless DC(BLDC)| 31     | --          |  0     |
| Turret            | Rev Spark Max   | NEO Brushless 550 | 32     | --          |  1     |  5*5
| Flap              | Rev Smart Servo | NEO Brushless     | ?      | --          |  ?     |
| Intake            | Rev Spark Flex  | NEO Brushless     | 40     | --          | ?      |
| Intake Lift(Left) | Rev Spark Max   | NEO Brushless     | 41     | Left        | ?      |
| IntakeLift(Right) | Rev Spark Max   | NEO Brushless     | 42     | Right       | ?      |
| Hopper (Upper)    | Rev Spark Flex  | NEO Brushless     | 50     | --          | ?      |
| Hopper (Lower)    | Rev Spark Flex  | NEO Brushless     | 51     | --          | ?      |
| Feeder (Leader)   | Rev Spark Flex  | NEO Brushless     | 52     | --          | ?      |
| Feeder (Follower) | Rev Spark Max   | NEO Brushless     | 53     | --          | ?      |
| Launcher (Leader) | Kraken x60      | Brushless DC(BLDC)| 60     | Right       | ?      |
| Launcher(Follower)| Kraken x61      | Brushless DC(BLDC)| 61     | Left        | ?      |

## Modules

| Module         | Module Type      | CAN ID | Position    | PDH ID |
| -------------- | ---------------- | ------ | ----------- | ------ |
| Power          | Rev Robotics PDB | 1      |             |        |
| Processor      | NI RoboRIO V2    | 0      |             | 21?    |
| Radio Power    |                  |        |             | 20?    |
|                |                  |        |             |        |

## Sensors

| Subsystem      | Mechanism Type   | Sensor Type        | CAN ID    | PDH ID |
| -------------- | ---------------- | ------------------ | --------- | ------ |
| Drive          | Gyro             | Pigeon             | 10        | 22?    |
| Chassis        | Vision           | Limelight          | --        |  7     |


| Subsystem      | Mechanism Type   | Sensor Type        | Port      | Position    |
| -------------- | ---------------- | ------------------ | --------- | ----------- |
| Lift           | Absolute Encoder |                    |           |             |
| Shoulder       | Absolute Encoder |                    |           |             |
|                |                  |                    |           |             |
