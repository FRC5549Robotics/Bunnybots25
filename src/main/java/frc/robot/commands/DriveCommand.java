package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List; 

public class DriveCommand extends Command {

    private double xDot;
    private double yDot;
    private double thetaDot;
    private boolean fieldRelative;
    private ChassisSpeeds chassisSpeeds, chassisPercent;
    private CommandXboxController m_controller;

    // The subsystem the command runs on
    public final 
    DrivetrainSubsystem drivetrain;
    Elevator m_elevator;
    Limelight m_Limelight;

    public DriveCommand(DrivetrainSubsystem subsystem, CommandXboxController controller, Elevator elevator, Limelight limelight){
        drivetrain = subsystem;
        m_controller = controller;
        m_elevator = elevator;
        m_Limelight = limelight;
        addRequirements(drivetrain);
    }
 
    @Override
    public void initialize() {
    }

            
    @Override
    public void execute() {
      double[] controllerVals = {m_controller.getLeftY(), m_controller.getLeftX(), m_controller.getRightX()};
        if (m_controller.leftBumper().getAsBoolean() || m_controller.rightBumper().getAsBoolean()) {
          boolean[] bool_values = {true, true, false};
          List<Double> speeds = DrivetrainSubsystem.generateSpeeds(bool_values, controllerVals);
          xDot = speeds.get(0);
          yDot = speeds.get(1);
          // thetaDot = m_Limelight.turnToTarget();

          // double[] dots = m_controller.rightBumper().getAsBoolean() ? m_Limelight.turnToTarget(true) : m_Limelight.turnToTarget(false);
          
          // if (dots != null) {
            // chassisSpeeds = new ChassisSpeeds(dots[0], dots[1], dots[2]);

          //   drivetrain.drive(chassisSpeeds, true);

          // }
        }

        else{
          boolean[] bool_values = {true, true, true};
          List<Double> speeds = DrivetrainSubsystem.generateSpeeds(bool_values, controllerVals);
          xDot = speeds.get(0);
          yDot = speeds.get(1);
          thetaDot = speeds.get(2);
           
          fieldRelative = true;
        
          // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, drivetrain.getHeading());
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, drivetrain.getHeading());
          if(xDot != 0 || yDot != 0 || thetaDot != 0){
            drivetrain.drive(chassisSpeeds, true);
          }
          else{
            drivetrain.snap();
          }
  
        }
              double[] scaledVals = new double[controllerVals.length];
        for (int i = 0; i < controllerVals.length; i ++) {
          scaledVals[i] = ((controllerVals[i]*(((48-m_elevator.getLeftElevatorPosition())*0.01)+0.52)));
        }  
       

    }
}