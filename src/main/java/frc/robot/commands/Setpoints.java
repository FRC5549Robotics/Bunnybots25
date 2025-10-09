// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotTarget;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Setpoints extends Command {
  /** Creates a new PivotSetpoint. */
  Pivot m_pivot;
  Pivot.PivotTarget target;
  double pivotSetpoint, elevatorLeftSetpoint, elevatorRightSetpoint;
  boolean end;
  Elevator elevator;

  public Setpoints(Pivot pivot, Pivot.PivotTarget Target, Elevator Elevator) {
    m_pivot = pivot;
    target = Target;
    elevator = Elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(target == PivotTarget.Intake){
      pivotSetpoint = Constants.PIVOT_INTAKE_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_INTAKE_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_INTAKE_SETPOINT;
    }
    else if(target == PivotTarget.Stowed){
      pivotSetpoint = Constants.PIVOT_STOWED_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_STOWED_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_STOWED_SETPOINT;
    }
    else if(target == PivotTarget.L1) {
      pivotSetpoint = Constants.PIVOT_L1_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_L1_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_L1_SETPOINT;
    }
    else if(target == PivotTarget.L2) {
      pivotSetpoint = Constants.PIVOT_L2_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_L2_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_L2_SETPOINT;
    }
    else if(target == PivotTarget.L3) {
      pivotSetpoint = Constants.PIVOT_L3_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_L3_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_L3_SETPOINT;
    }
    // else if(target == PivotTarget.L4) {
    //   pivotSetpoint = Constants.PIVOT_L4_SETPOINT;
    //   elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_L4_SETPOINT;
    //   elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_L4_SETPOINT;
    // }
    else if(target == PivotTarget.AlgaeLow){
      pivotSetpoint = Constants.PIVOT_ALGAE_LOW_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_ALGAE_LOW_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_ALGAE_LOW_SETPOINT;
    }
    else if(target == PivotTarget.AlgaeHigh){
      pivotSetpoint = Constants.PIVOT_ALGAE_HIGH_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_ALGAE_HIGH_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_ALGAE_HIGH_SETPOINT;
    }
    else if(target == PivotTarget.Processor){
      pivotSetpoint = Constants.PIVOT_PROCESSOR_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_PROCESSOR_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_PROCESSOR_SETPOINT;
    }
    else if (target == PivotTarget.Climb){
      pivotSetpoint = Constants.PIVOT_CLIMB_SETPOINT;
      elevatorLeftSetpoint = Constants.ELEVATOR_LEFT_STOWED_SETPOINT;
      elevatorRightSetpoint = Constants.ELEVATOR_RIGHT_STOWED_SETPOINT;
    }
  }

  boolean pivotState(Trigger[] buttons){
    for (Trigger trigger : buttons) {
      if(trigger.getAsBoolean()){
        return false;
      }
    }
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.PivotToSetpoint(pivotSetpoint);
    elevator.ElevateToSetpoint(elevatorLeftSetpoint, elevatorRightSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.off(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
