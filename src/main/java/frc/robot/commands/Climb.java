// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new ElevateAnalog. */
  Climber m_climber;
  CommandXboxController m_controller;
  Pivot m_pivot;
  public Climb(Climber climber, Pivot pivot, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_controller = controller;
    m_pivot  = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.climb();
    m_pivot.PivotToSetpoint(Constants.PIVOT_PROCESSOR_SETPOINT);
    
    // m_climber.climb(m_controller.getLeftY()*Constants.CLIMBER_SCALING_FACTOR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
