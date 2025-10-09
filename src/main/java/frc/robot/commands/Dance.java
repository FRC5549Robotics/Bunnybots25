// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem.direction;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Dance extends Command {
  /** Creates a new DanceAuton. */
  DrivetrainSubsystem m_drive;
  Timer timer = new Timer();
  DrivetrainSubsystem.direction m_direction;


  public Dance(DrivetrainSubsystem drive, DrivetrainSubsystem.direction direction) {
    m_drive = drive;
    m_direction = direction;
    System.out.println("Dance command called.");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    // System.out.println("Timer initialized.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_direction == direction.left) {
      m_drive.drive(new ChassisSpeeds(0, -0.7, 0), true);
    }
    if (m_direction == direction.right) {
      m_drive.drive(new ChassisSpeeds(0, 0.7, 0), true);
    }
    if (m_direction == direction.backward) {
      m_drive.drive(new ChassisSpeeds(0.7, 1, 0), true);
    }
    if (m_direction == direction.forward) {
      m_drive.drive(new ChassisSpeeds(-0.7, 0, 0), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get()>2);
  }
  }



