// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  // PhotonCamera camera;
  DrivetrainSubsystem m_drivetrain;
  CommandXboxController xbox_controller;
  PIDController controller = new PIDController(0.1, 0, 0.001);
  PIDController controller2 = new PIDController(2, 0, 0.002);
  NetworkTable limelightTable;

  public Limelight(DrivetrainSubsystem drivetrain, CommandXboxController xcontroller) {
    m_drivetrain = drivetrain;
    xbox_controller = xcontroller;
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  // public double[] turnToTarget(Boolean isRightScore) {
    // thetaController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    // thetaController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    // xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    // xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    // yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_RIGHT_REEF_ALIGNMENT : Constants.Y_SETPOINT_LEFT_REEF_ALIGNMENT);
    // yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

  //   if (LimelightHelpers.getTV("limelight")) {
  //     double[] s = LimelightHelpers.getBotPose_TargetSpace("limelight");
  //     // Pose3d bot = LimelightHelpers.getBotPose3d_wpiBlue("limelight");
  //     Pose3d ttr = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  //     double[] speeds = {xController.calculate(ttr.getZ()), yController.calculate(ttr.getX()), thetaController.calculate(s[4])};
  //     return speeds;
  //   }


  //   double[] s = LimelightHelpers.getBotPose_TargetSpace("limelight");
  //     // Pose3d bot = LimelightHelpers.getBotPose3d_wpiBlue("limelight");
  //   Pose3d ttr = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  //   double angle = s[4] - 4.6;
    
  //   // if(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0){
  //   //   if (xbox_controller.getHID().getLeftBumperButton()){
  //   //     // System.out.println(controller2.calculate(0, -ttr.getX()-0.18));
  //   //     // System.out.println(controller.calculate(angle, 0));
  //   //     double[] speeds = {controller2.calculate(0, ttr.getZ()+0.45), controller2.calculate(0, -ttr.getX()-.24), controller.calculate(angle, 0)};
  //   //     return speeds;
        
  //   //   }
  //   //   else if (xbox_controller.getHID().getRightBumperButton()) {
  //   //     double[] speeds = {controller2.calculate(0, ttr.getZ()+0.45), controller2.calculate(0, -ttr.getX()+.02), controller.calculate(angle, 0)};


      
  //   //     return speeds;
  //    }
  //   }
  //   return null;
  // }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double[] s = LimelightHelpers.getBotPose_TargetSpace("limelight");
    // System.out.println(s);
    // var s = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    // System.out.println(s);

    // for (int i = 0; i<s.length; i++) {
    //   System.out.println(s[i]);
    // }
    // System.out.println(s[4]);
  }}

