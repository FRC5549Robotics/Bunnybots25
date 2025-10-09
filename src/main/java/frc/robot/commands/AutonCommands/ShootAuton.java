// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shintake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAuton extends Command {
  /** Creates a new ShootAuton. */
  Shintake m_shintake;
  Double m_time;
  Timer timer = new Timer();
  public ShootAuton(Shintake shintake, Double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shintake = shintake;
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shintake.shootL1();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shintake.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get()>m_time);
  }
}
