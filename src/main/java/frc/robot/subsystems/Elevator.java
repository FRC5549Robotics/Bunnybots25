// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  TalonFX ElevatorRightMotor;
  TalonFX ElevatorLeftMotor;
  ProfiledPIDController ElevatorController;
  DutyCycleEncoder ElevatorThroughbore;
  CommandXboxController XboxController;
  TalonFXConfiguration ElevatorRightConfigs;
  TalonFXConfigurator ElevatorRightConfigurator;
  TalonFXConfiguration ElevatorLeftConfigs;
  TalonFXConfigurator ElevatorLeftConfigurator;
  Trigger[] setpointButtons;
  boolean reset = true;
  
  public Elevator(CommandXboxController xboxController, Trigger[] SetpointButtons) {
    XboxController = xboxController;
    setpointButtons = SetpointButtons;
    ElevatorLeftMotor = new TalonFX(Constants.ELEVATOR_LEFT_MOTOR);
    ElevatorRightMotor = new TalonFX(Constants.ELEVATOR_RIGHT_MOTOR);
    //region Configs
    ElevatorLeftConfigs = new TalonFXConfiguration();
    ElevatorRightConfigs = new TalonFXConfiguration();
    ElevatorLeftConfigurator = ElevatorLeftMotor.getConfigurator();
    ElevatorRightConfigurator = ElevatorRightMotor.getConfigurator();
    ElevatorLeftConfigs.CurrentLimits.StatorCurrentLimit = 60;
    ElevatorLeftConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    ElevatorLeftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ElevatorRightConfigs.CurrentLimits.StatorCurrentLimit = 60;
    ElevatorRightConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    ElevatorRightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ElevatorLeftConfigurator.apply(ElevatorLeftConfigs);
    ElevatorRightConfigurator.apply(ElevatorRightConfigs);
    //endregion
    // ElevatorController = new PIDController(0.06, 0.0, 0.0);
    ElevatorController = new ProfiledPIDController(0.03, 0.0, 0.0, new Constraints(-1, -0.1));
  }

  public void elevate(double speed){
    ElevatorLeftMotor.set(speed);
    ElevatorRightMotor.set(-speed);
  }

  public void off(){
    ElevatorLeftMotor.set(0);
    ElevatorRightMotor.set(0);
  }

  public void ElevateToSetpoint(double leftElevatorSetpoint, double rightElevatorSetpoint) {
    ElevatorLeftMotor.set(ElevatorController.calculate(getLeftElevatorPosition(), leftElevatorSetpoint)); 
    ElevatorRightMotor.set(ElevatorController.calculate(getRightElevatorPosition(), rightElevatorSetpoint));
  }

  public double getLeftElevatorPosition() {
    return ElevatorLeftMotor.getPosition().getValueAsDouble(); 
  }
  public double getRightElevatorPosition() {
    return ElevatorRightMotor.getPosition().getValueAsDouble(); 
  }

  public double detectElevatorLeftCurrent() {
    return ElevatorLeftMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double detectElevatorRightCurrent(){
    return ElevatorRightMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void ResetEncoder() {
    ElevatorLeftMotor.setPosition(0);
    ElevatorRightMotor.setPosition(0);
  }

  public void Snapback() {
    if(elevatorState(setpointButtons) && !reset){
      ElevatorLeftMotor.set(ElevatorController.calculate(getLeftElevatorPosition(), Constants.ELEVATOR_LEFT_STOWED_SETPOINT)); 
      ElevatorRightMotor.set(ElevatorController.calculate(getRightElevatorPosition(), Constants.ELEVATOR_RIGHT_STOWED_SETPOINT));
    }
    // if (Math.abs(getLeftElevatorPosition()) < 2 && Math.abs(getRightElevatorPosition()) < 2) {
    //   ElevatorLeftMotor.set(0);
    //   ElevatorRightMotor.set(0);
    // }
  }

  boolean elevatorState(Trigger[] buttons){
    for (Trigger trigger : buttons) {
      if(trigger.getAsBoolean()){
        return false;
      }
    }
    return true;
  }

  public void reset() {
    if (detectElevatorLeftCurrent() >= Constants.ELEVATOR_RESET_CURRENT && detectElevatorRightCurrent() >= Constants.ELEVATOR_RESET_CURRENT){
      ElevatorLeftMotor.setPosition(0);
      ElevatorRightMotor.setPosition(0);
      reset = false;
      off();
    }
    else{
      ElevatorLeftMotor.set(-0.2);
      ElevatorRightMotor.set(0.2);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (reset){
      if (detectElevatorLeftCurrent() >= Constants.ELEVATOR_RESET_CURRENT && detectElevatorRightCurrent() >= Constants.ELEVATOR_RESET_CURRENT){
        ElevatorLeftMotor.setPosition(0);
        ElevatorRightMotor.setPosition(0);
        reset = false;
        off();
      }
      else{
        ElevatorLeftMotor.set(-0.2);
        ElevatorRightMotor.set(0.2);
      }
    }

    SmartDashboard.putNumber("LeftElevatorEncoder", getLeftElevatorPosition());
    SmartDashboard.putNumber("RightElevatorEncoder", getRightElevatorPosition());
  }
}
