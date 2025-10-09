// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAuton;
import frc.robot.commands.HardcodedDrive;
import frc.robot.commands.Setpoints;
import frc.robot.commands.SetpointsAuton;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Pivot.PivotTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HardcodedAuton extends SequentialCommandGroup {
  /** Creates a new HardcodedAuton. */
  public HardcodedAuton(DrivetrainSubsystem drivetrain, Pivot pivot, Elevator elevator, Shintake shintake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(drivetrain::resetOdometry),
    new WaitCommand(1),
    // new HardcodedDrive(drivetrain),
    new SetpointsAuton(pivot, PivotTarget.L1, elevator),
    new HardcodedDrive(drivetrain),
    new SetpointsAuton(pivot, PivotTarget.L1, elevator),
    new ShootAuton(shintake, 1.5),
    new InstantCommand(shintake::off),
    new SetpointsAuton(pivot, PivotTarget.Stowed, elevator)
    );
  }
}
