// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  SparkMax ClimberMotor;

  /** Creates a new Pivot. */
  public Climber() {
    ClimberMotor = new SparkMax(Constants.CLIMBER_MOTOR, MotorType.kBrushless);
    SparkMaxConfig Climberconfig = new SparkMaxConfig();
    Climberconfig.idleMode(IdleMode.kBrake);
    ClimberMotor.configure(Climberconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void climb(){
    ClimberMotor.set(1);
  }
  public void unwind(){
    ClimberMotor.set(-0.6);
  }

  public void off(){
    ClimberMotor.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}