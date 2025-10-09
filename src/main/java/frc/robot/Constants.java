// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int DRIVE_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 13; 
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; 
  public static final double FRONT_LEFT_MAGNET_OFFSET = -0.002197; 

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;  
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13; 
  public static final double FRONT_RIGHT_MAGNET_OFFSET = -0.547852; 

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; 
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; 
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
  public static final double BACK_LEFT_MAGNET_OFFSET = -0.314209; 

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; 
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9; 
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
  public static final double BACK_RIGHT_MAGNET_OFFSET = -0.610840;  

  // Define the conventional order of our modules when putting them into arrays
  public static final int FRONT_LEFT =0;
  public static final int FRONT_RIGHT =1;
  public static final int REAR_LEFT =2;
  public static final int REAR_RIGHT =3;

  public static final boolean kFrontLeftDriveEncoderReversed = false;
  public static final boolean kFrontRightDriveEncoderReversed = false;
  public static final boolean kRearLeftDriveEncoderReversed = true;
  public static final boolean kRearRightDriveEncoderReversed = true;

  public static final boolean kFrontLeftTurningEncoderReversed = false;
  public static final boolean kFrontRightTurningEncoderReversed = false;
  public static final boolean kRearLeftTurningEncoderReversed = true;
  public static final boolean kRearRightTurningEncoderReversed = true;

  public static final double kWheelDiameterMeters = 0.1016; //0.098; // 0.09398; // 3.7 in

  // The drive encoder reports in RPM by default. Calculate the conversion factor
  // to make it report in meters per second.
  public static final double kDriveGearRatio = 8.143;
  public static final double kDriveConversionFactor = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

  public static final double kTurnPositionConversionFactor = 12.8;

  public static final double kMaxSpeedMetersPerSecond = 4;
  // Units are meters.
  // Distance between centers of right and left wheels on robot
  public static final double kTrackWidth = 0.51435;
  
  // Distance between front and back wheels on robot
  public static final double kWheelBase = 0.51435;

  // Units are meters per second
  public static final double kMaxTranslationalVelocity = 6784 / 60.0 *
  (1/kDriveGearRatio) *
  kWheelDiameterMeters * Math.PI;

  // Units are radians per second
  public static final double kMaxRotationalVelocity = kMaxTranslationalVelocity /
  Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);; //max 5.0

  //The locations f
  //*or the modules must be relative to the center of the robot. 
  // Positive x values represent moving toward the front of the robot 
  // Positive y values represent moving toward the left of the robot.
  public static final SwerveDriveKinematics kDriveKinematics =
  new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),   // front left
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),  // front right
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),  // rear left
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)  // rear right
      );


      
  public static final boolean kGyroReversed = false;

  public static final double kDriveP = 0.000000000000001; //Usually 0.05, 0.000000000000001?
  public static final double kDriveI = 0.0;
  public static final double kDriveD = 0.9; //Usually 0.0, 0.9;?
  public static final double kDriveFF = 0.1;
  public static final double SLEW_RATE_LIMITER = 3;

  public static final double kTurningP = 0.005; //Usually 0.05
  public static final double kTurningI = 0.0;
  public static final double kTurningD = 0.05;
  public static final double kAcceleration = 4;

  public static final int RESET_NAVX_BUTTON = 8;

  public static final Mode simMode = Mode.REAL;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  //Elevator Constants
    public static final int ELEVATOR_LEFT_MOTOR = 30;
    public static final int ELEVATOR_RIGHT_MOTOR = 31;
    public static final int ELEVATOR_JOYSTICK = 1;
    public static final double ELEVATOR_DEADBAND = 0.05;
    public static final double ELEVATOR_SCALING_FACTOR = -0.2;
    public static final int ELEVATOR_ENCODER = 8;
    public static final double ELEVATOR_OFFSET = 0.331;  

    public static final double ELEVATOR_LEFT_INTAKE_SETPOINT = 13.212891;
    public static final double ELEVATOR_LEFT_STOWED_SETPOINT = 0.385254;
    public static final double ELEVATOR_LEFT_L1_SETPOINT = 10.768750; 
    public static final double ELEVATOR_LEFT_L2_SETPOINT = 22.015137;
    public static final double ELEVATOR_LEFT_L3_SETPOINT = 40.010254; //previously 39.010254
    // public static final double ELEVATOR_LEFT_L4_SETPOINT = 0;
    public static final double ELEVATOR_LEFT_ALGAE_LOW_SETPOINT = 25.516602; 
    public static final double ELEVATOR_LEFT_ALGAE_HIGH_SETPOINT = 46.582520;
    public static final double ELEVATOR_LEFT_PROCESSOR_SETPOINT = 0.285254;

    public static final double ELEVATOR_RIGHT_INTAKE_SETPOINT = -13.25820;
    public static final double ELEVATOR_RIGHT_STOWED_SETPOINT = -0.442383;
    public static final double ELEVATOR_RIGHT_L1_SETPOINT = -9.911133;
    public static final double ELEVATOR_RIGHT_L2_SETPOINT = -24.961563;
    public static final double ELEVATOR_RIGHT_L3_SETPOINT = -42.04;
    
    // public static final double ELEVATOR_RIGHT_L4_SETPOINT = 0;
    public static final double ELEVATOR_RIGHT_ALGAE_LOW_SETPOINT = -25.616211;  
    public static final double ELEVATOR_RIGHT_ALGAE_HIGH_SETPOINT = -46.680176;
    public static final double ELEVATOR_RIGHT_PROCESSOR_SETPOINT = -0.442383;

    public static final double ELEVATOR_RESET_CURRENT = 3;
    public static final int RESET_ENCODER_BUTTON = 7;
  
    //Pivot Constants
    public static final int PIVOT_MOTOR = 32;
    public static final int PIVOT_JOYSTICK = 5;
    public static final double PIVOT_DEADBAND = 0.05;
    public static final double PIVOT_SCALING_FACTOR = 0.3;
    public static final int PIVOT_ENCODER = 4;
    public static final double PIVOT_OFFSET = 0.331;

    public static final double PIVOT_INTAKE_SETPOINT = 9.84;
    public static final double PIVOT_STOWED_SETPOINT = 0;
    public static final double PIVOT_L1_SETPOINT = 21.735352;
    public static final double PIVOT_L2_SETPOINT = 24.019531;
    public static final double PIVOT_L3_SETPOINT = 24.235352; //previously 26.235352
    public static final double PIVOT_ALGAE_LOW_SETPOINT = 21.556152;
    public static final double PIVOT_ALGAE_HIGH_SETPOINT = 24.415527;
    public static final double PIVOT_CLIMB_SETPOINT = 28.8;
    public static final double PIVOT_MIN_THRESHOLD = 5;
    public static final double PIVOT_RESET_CURRENT = 4;
    public static final double PIVOT_PROCESSOR_SETPOINT = 17.29;

  //Shintake Constants
    public static final int INTAKE_MOTOR_LEFT = 33;
    public static final int INTAKE_MOTOR_RIGHT = 40;
    public static final int SHOOTER_INTAKE_BUTTON = 6;
    public static final double INTAKE_OUTTAKE_SPEED = 1;
    public static final double OUTTAKE_L1_SPEED = 0.4;
    public static final int SHINTAKE_BUTTON = 0;
    public static final int INTAKE_TRIGGER = 3;
    public static final int OUTTAKE_TRIGGER = 2;//RETURN TO OLD SETTING LATER


  //Climb Constants
    public static final int CLIMBER_MOTOR = 24;
    public static final int CLIMBER_JOYSTICK = 1;
    public static final double CLIMBER_DEADBAND = 0.05;
    public static final double CLIMBER_SCALING_FACTOR = 0.15;
    public static final int CLIMB_BUTTON = 7;
    public static final int CLIMB_UNWIND = 8;

  //Limelight Constants
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.0;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1.0;
    public static final double X_SETPOINT_REEF_ALIGNMENT = 0.3;
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.01;    
    public static final double Y_SETPOINT_RIGHT_REEF_ALIGNMENT = 0.0;
    public static final double Y_SETPOINT_LEFT_REEF_ALIGNMENT = 0.0;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.01;
}
