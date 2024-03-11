// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final int JOYSTICK_DRIVER = 0;
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double DRIVE_THROTTLE = 0.8;
    public static final double DRIVE_TURBO = 1.0;
    public static final int ARCADE_DRIVE_TURBO = 6;
    public static final double MAX_SPEED = 14.5;  // default 14.5
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final int JOYSTICK_OPERATOR = 1;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int INTAKE_MOTOR_PORT = 12;
    public static final int INDEXER_MOTOR_PORT = 11;
    public static final int SHOOTER_MOTOR_PORT = 10;
    public static final int ELEVATOR_MOTOR1_PORT = 20;
    public static final int ELEVATOR_MOTOR2_PORT = 21;
    public static final int CLIMBER_MOTOR_PORT = 30;
    public static final int INTAKE_IN = 2; // left trigger
    public static final int INTAKE_OUT = 3; // right trigger
    public static final int SHOOTER = 2;
    public static final int ELEVATOR_AXIS = 1;
    public static final int ELEVATOR_HIGH = 3;  //PID!!!
    public static final int ELEVATOR_LOW = 4;  //PID!!!
    public static final int CLIMBER_AXIS = 5;
    public static final int INDEXER_FORWARD = 5;
    public static final int INDEXER_REVERSE = 6;

  }

  public static class OperatorPIDConstants
  {
    public static final double ELEVATOR_P = 0.01;  // NEED TO TEST THESE!!!
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 20;
    public static final double ELEVATOR_MIN_HEIGHT = 0;
  }
}
