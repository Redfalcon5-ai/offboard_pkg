#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class OffboardTakeoffLand(Node):
    def __init__(self):
        super().__init__('offboard_takeoff_land_node')
        
        # Publishers for offboard mode, trajectory setpoints, and vehicle commands
        self.offboard_control_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', 10)
        
        # In NED, altitude is negative upward. For example, a 3m takeoff uses -3.0.
        self.takeoff_altitude = -3.0
        self.landing_altitude = 0.0

        # Two flight phases: TAKEOFF then LANDING.
        self.phase = "TAKEOFF"
        self.offboard_mode_set = False
        self.landing_command_sent = False
        
        # Counter for the number of setpoints sent.
        self.setpoints_sent = 0
        
        # Timer callback at 20 Hz.
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # 1. Publish OffboardControlMode continuously.
        offb_mode = OffboardControlMode()
        offb_mode.position = True
        offb_mode.velocity = False
        offb_mode.acceleration = False
        offb_mode.attitude = False
        offb_mode.body_rate = False
        self.offboard_control_pub.publish(offb_mode)
        
        # 2. Publish TrajectorySetpoint.
        traj_sp = TrajectorySetpoint()
        traj_sp.position[0] = 0.0
        traj_sp.position[1] = 0.0
        if self.phase == "TAKEOFF":
            traj_sp.position[2] = self.takeoff_altitude  # Ascend to 3m (NED: -3.0)
        elif self.phase == "LANDING":
            traj_sp.position[2] = self.landing_altitude   # Descend to ground level (0.0)
        self.trajectory_setpoint_pub.publish(traj_sp)
        
        self.setpoints_sent += 1
        
        # 3. After a short delay, switch to offboard mode and arm the drone.
        if self.setpoints_sent == 20 and not self.offboard_mode_set:
            self.get_logger().info("Setting Offboard mode and arming the drone...")
            # Command to set OFFBOARD mode.
            mode_cmd = VehicleCommand()
            mode_cmd.command = 176       # MAV_CMD_DO_SET_MODE
            mode_cmd.param1 = 1.0        # Base mode (use appropriate value per your setup)
            mode_cmd.param2 = 6.0        # Custom mode: OFFBOARD (commonly 6 in PX4)
            mode_cmd.target_system = 1
            mode_cmd.target_component = 1
            mode_cmd.source_system = 1
            mode_cmd.source_component = 1
            mode_cmd.from_external = True
            self.vehicle_command_pub.publish(mode_cmd)
            
            # Command to arm the drone.
            arm_cmd = VehicleCommand()
            arm_cmd.command = 400        # MAV_CMD_COMPONENT_ARM_DISARM
            arm_cmd.param1 = 1.0         # 1 to arm
            arm_cmd.target_system = 1
            arm_cmd.target_component = 1
            arm_cmd.source_system = 1
            arm_cmd.source_component = 1
            arm_cmd.from_external = True
            self.vehicle_command_pub.publish(arm_cmd)
            
            self.offboard_mode_set = True

        # 4. After about 10 seconds (200 iterations), switch to landing.
        if self.setpoints_sent == 200 and self.phase == "TAKEOFF":
            self.get_logger().info("Switching to landing phase...")
            self.phase = "LANDING"
            # Send a landing command (MAV_CMD_NAV_LAND).
            if not self.landing_command_sent:
                land_cmd = VehicleCommand()
                land_cmd.command = 21      # MAV_CMD_NAV_LAND
                land_cmd.param1 = 0.0      # Optional parameters can be set if needed
                land_cmd.param2 = 0.0
                land_cmd.target_system = 1
                land_cmd.target_component = 1
                land_cmd.source_system = 1
                land_cmd.source_component = 1
                land_cmd.from_external = True
                self.vehicle_command_pub.publish(land_cmd)
                self.landing_command_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffLand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OffboardTakeoffLand node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

