// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class Pivot extends SubsystemBase {
  public enum PivotTarget{
    Stowed,
    Intake,
    L1,
    L2,
    L3,
    L4,
    AlgaeLow,
    AlgaeHigh,
    Processor,
    Climb
  }
  TalonFX PivotMotor;
  PIDController PivotController;
  DutyCycleEncoder PivotThroughbore;
  CommandXboxController XboxController;
  TalonFXConfiguration PivotConfigs;
  TalonFXConfigurator PivotConfigurator;
  boolean intakePosition = false;
  boolean lock = true;
  boolean reset = true;
  DutyCycleEncoder Throughbore;
  Trigger[] setpointButtons;
  Elevator m_elevator;

  /** Creates a new Pivot. */
  public Pivot(CommandXboxController xboxController, Trigger[] SetpointButtons) {
    setpointButtons = SetpointButtons;
    XboxController = xboxController;
    PivotMotor = new TalonFX(Constants.PIVOT_MOTOR);
    //region Configs
    PivotConfigs = new TalonFXConfiguration();
    PivotConfigurator = PivotMotor.getConfigurator();
    PivotConfigs.CurrentLimits.StatorCurrentLimit = 60;
    PivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    PivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PivotConfigurator.apply(PivotConfigs);
    // Throughbore = new DutyCycleEncoder(0, 360, Constants.PIVOT_OFFSET);
    
    

    //endregion
    PivotController = new PIDController(0.04, 0.0, 0.005);
    
  }

  public void pivot(double speed){
    PivotMotor.set(speed);
  }

  public void off(){
    PivotMotor.set(0);
  }

  public double getPivotPosition() {
    return PivotMotor.getPosition().getValueAsDouble(); 
    // return Throughbore.get();
  }

  public void PivotToSetpoint(double pivotSetpoint) {
    PivotMotor.set(PivotController.calculate(getPivotPosition(), pivotSetpoint)); 
  }

  public double detectPivotMotorCurrent() {
    return PivotMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void ResetEncoder() {
    PivotMotor.setPosition(0);
  }

  public void Snapback() {
    if(pivotState(setpointButtons) && !reset){
      PivotMotor.set(PivotController.calculate(getPivotPosition(), Constants.PIVOT_STOWED_SETPOINT));
    }
    // if (Math.abs(getPivotPosition()) < 2) {
    //   PivotMotor.set(0);
    // }
  }

  boolean pivotState(Trigger[] buttons){
    for (Trigger trigger : buttons) {
      if(trigger.getAsBoolean()){
        return false;
      }
    }
    return true;
  }

  public void reset() {
    if (detectPivotMotorCurrent() >= Constants.PIVOT_RESET_CURRENT){
      PivotMotor.setPosition(0);
      reset = false;
      off();
    }
    else{
      PivotMotor.set(-0.2);
    }
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PivotEncoder", getPivotPosition());
   
    if (reset){
      if (detectPivotMotorCurrent() >= Constants.PIVOT_RESET_CURRENT){
        PivotMotor.setPosition(0);
        reset = false;
        off();
      }
      else{
        PivotMotor.set(-0.2);
      }
    }
  }
}
