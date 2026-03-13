// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static final int GYRO_CAN_ID = 18;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

    public static final double DEFAULT_ROBOT_SPEED = .9;


    public static final double SLOW_SPEED_SCALE = .7;

    public static final class OdometryConstants {
      public static final double TRACK_WIDTH_METERS = 0.546;
      public static final double DRIVE_GEAR_RATIO = 1/8.46;
      public static final double DRIVE_WHEEL_CIRCUMFERENCE = 18.8495559215;
      
    }
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final class CanIds {
      public static final int INTAKE_MOTOR_ID = 8;
      public static final int FEEDER_MOTOR_ID = 6;
      public static final int LAUNCHER_MOTOR_ID = 5;
    }

    public static final class CurrentLimits{
      // Current limit and nominal voltage for fuel mechanism motors.
      public static final int FEEDER_MOTOR_CURRENT_LIMIT = 30;
      public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;
      public static final int INTAKE_MOTOR_CURRENT_LIMIT = 30;
    }
    

    

    



    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = 6;
    public static final double INTAKING_INTAKE_VOLTAGE = -6;
    public static final double LAUNCHING_INTAKE_VOLTAGE = -6;
    public static final double LAUNCHING_FEEDER_VOLTAGE = -7;

    public static final double LAUNCHING_LAUNCHER_RPM = 2300; //Original: 2300
    public static final double SLOW_LAUNCHING_LAUNCHER_RPM = 2100;
    public static final double INTAKING_LAUNCHER_RPM = 0;

    public static final double SPIN_UP_FEEDER_VOLTAGE = 1;
    public static final double SPIN_UP_SECONDS = 0.2;

    public static final class LauncherConstants{
      public static final double LAUNCHER_kP = 0.00025;
      public static final double LAUNCHER_kI = 0.0;
      public static final double LAUNCHER_kD = 0.00015;

      public static final double LAUNCHER_kMinOutput = -1;
      public static final double LAUNCHER_kMaxOutput = 1;

      public static final double LAUNCHER_MIN_RPM = -6000;
      public static final double LAUNCHER_MAX_RPM = 6000;
    }

  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 1;
    public static final double ROTATION_SCALING = .9;
  }

  public static final class PathPlannerConstants {

    public static final RobotConfig ROBOT_CONFIG;
    static {
      try {
        ROBOT_CONFIG = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        throw new RuntimeException(
        "Failed to load PathPlanner RobotConfig from GUI settings!",
        e
        );
      }
    }
  }

  public static final class VisionConstants {
    public static final boolean USE_LIMELIGHT_BL = true;
    public static final boolean USE_LIMELIGHT_FR = true;

    public static final boolean USE_MEGA_TAG_1 = true;
    public static final boolean USE_MEGA_TAG_2 = false;

  
    
  }

}

