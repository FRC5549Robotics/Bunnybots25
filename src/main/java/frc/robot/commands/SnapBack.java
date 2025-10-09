// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapBack extends Command {
  /** Creates a new SnapBack. */
  Pivot m_pivot;
  Elevator m_elevator;
  Boolean ending = false;
  public SnapBack(Pivot pivot, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
    m_elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ending = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_elevator.getRightElevatorPosition()) < 5 && Math.abs(m_elevator.getLeftElevatorPosition()) < 5) {
      System.out.println("SNAPBACK ALL");
      m_pivot.Snapback();
      m_elevator.Snapback();
    }
    else {
      System.out.println("SNAPBACK E");
      m_elevator.Snapback();
    }

    if (Math.abs(m_elevator.getLeftElevatorPosition()) < 5 && Math.abs(m_elevator.getRightElevatorPosition()) < 5 && Math.abs(m_pivot.getPivotPosition()) < 5) {
      ending = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.off();
    m_elevator.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ending;
  }
}
