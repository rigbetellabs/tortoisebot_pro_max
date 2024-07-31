#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class DiagnosticsTest(Node):
    def __init__(self):
        super().__init__('diagnostics_test')
        self.publisher_ = self.create_publisher(Int32, '/diagnostics/test', 10)
        self.run_diagnostics()

    def run_diagnostics(self):
        prompts = {
            1: "Running motor and encoder connections/diagnostics test, Please confirm your robot is uplifted and emergency switch is open (y/n): ",
            2: "Do you want to run motor direction test? (y/n): ",
            3: "Do you want to run IMU connections/diagnostics test? (y/n): ",
            4: "Do you want to run display connection/diagnostics test? (y/n): ",
            5: "Do you want to publish 5? (y/n): ",
            6: "Do you want to publish 6? (y/n): ",
            7: "Do you want to publish 6? (y/n): "
        }

        print("\n\ProMAX diagnostics test\n")
        print(" *******************************************\n")
        print(" 1: To run Motor and encoder connections/diagnostics test ")
        print(" 2: To run motor direction test ")
        print(" 3: To run IMU connections/diagnostics test ")
        print(" 4: To run display connection/diagnostics test ")
        print(" *******************************************")
        print("Enter test code and press Enter.")
        print("Press Ctrl+C to exit.\n")

        while rclpy.ok():
            try:
                user_input = int(input("Enter test code: "))
            except ValueError:
                print("Invalid input. Please enter a valid integer.")
                continue

            if user_input < 0 or user_input > 7:
                print("Input must be between 0 and 7.")
                continue

            confirmation = input(prompts[user_input]).lower()
            if confirmation == 'y' or confirmation == '':

                if user_input == 2:
                    # Robot forward direction test
                    print(" ")
                    print("Robot forward test: make sure robot is moving forward and all wheels rotate in forward direction")
                    feed_input = input("Are you ready to move forward? (y/n): ").lower()
                    if feed_input == 'y' or feed_input == '':
                        self.publisher_.publish(Int32(data=21))
                        time.sleep(5)
                        feed_input = input("Is robot/all motors moving forward? (y/n): ").lower()
                        if feed_input == 'y' or feed_input == '':
                            print("*** Great!! Now ready for backward test ***")
                            # Publish forward diagnosis result here
                        else:
                            print("Robot forward test: Failed")
                            # Publish forward diagnosis result here
                            continue

                    # Robot backward direction test
                    print(" ")
                    print("Robot backward test: make sure robot is moving backward and all wheels run in backward direction")
                    feed_input = input("Are you ready to move backward? (y/n): ").lower()
                    if feed_input == 'y' or feed_input == '':
                        self.publisher_.publish(Int32(data=22))
                        time.sleep(5)
                        feed_input = input("Is robot/all motors moving backward? (y/n): ").lower()
                        if feed_input == 'y' or feed_input == '':
                            print("*** Great!! Now ready for turning left test ***")
                            # Publish backward diagnosis result here
                        else:
                            print("Robot backward test: Failed")
                            # Publish backward diagnosis result here
                            continue

                    # Robot left direction test
                    print(" ")
                    print("Robot rotate left test: make sure robot is moving towards left direction and left wheels moving backward and right wheels moving to forward direction")
                    feed_input = input("Are you ready to rotate robot in left direction? (y/n): ").lower()
                    if feed_input == 'y' or feed_input == '':
                        self.publisher_.publish(Int32(data=23))
                        time.sleep(5)
                        feed_input = input("Is robot rotating to left direction? (y/n): ").lower()
                        if feed_input == 'y' or feed_input == '':
                            print("*** Great!! Now ready for right turning test ***")
                            # Publish left rotate diagnosis result here
                        else:
                            print("Robot rotate left test: Failed")
                            # Publish left rotate diagnosis result here
                            continue

                    # Robot right direction test
                    print(" ")
                    print("Robot rotate right test: make sure robot is moving towards right direction and right wheels moving backward and left wheels moving forward direction")
                    feed_input = input("Are you ready to rotate robot in right direction? (y/n): ").lower()
                    if feed_input == 'y' or feed_input == '':
                        self.publisher_.publish(Int32(data=24))
                        time.sleep(5)
                        feed_input = input("Is robot rotating to right direction? (y/n): ").lower()
                        if feed_input == 'y' or feed_input == '':
                            print("*** Great!! Test Successful ***")
                            # Publish right rotate diagnosis result here
                        else:
                            print("Robot rotate right test: Failed")
                            # Publish right rotate diagnosis result here
                            continue
                    else:
                        print("Motor Direction test: Aborted!!")
                else:
                    self.publisher_.publish(Int32(data=user_input))
            else:
                print("Publishing canceled.")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
