package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase{
    TalonFX pivotMotor;
    PIDController pivotPID;
    CommandXboxController Xboxcontroller;
    TalonFXConfiguration PivotConfigs;
    TalonFXConfigurator PivotConfigurator;
    ProfiledPIDController ElevatorController;

    SparkMax IntakeMotor;
    SparkMaxConfig IntakeConfig;

   
    public GroundIntake (CommandXboxController xboxController){
        Xboxcontroller = xboxController;
        pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID);
        PivotConfigs = new TalonFXConfiguration();
        PivotConfigurator = pivotMotor.getConfigurator();

        IntakeMotor = new SparkMax(Constants.GROUND_INTAKE_ID, MotorType.kBrushless);
        IntakeConfig = new SparkMaxConfig();
        IntakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40,40);
        IntakeMotor.configure(IntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);



        PivotConfigs.CurrentLimits.StatorCurrentLimit = 60;
        PivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        PivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        PivotConfigurator.apply(PivotConfigs);

        
    }

    public void pivotDown(){
        pivotMotor.set(0.5);
    }

    public void pivotUp(){
        pivotMotor.set(-0.5);
    }
    public void IntakeOn(){
        IntakeMotor.set(Constants.INTAKE_SPEED);
    }
    
    public void off (){
        pivotMotor.set(0);
        IntakeMotor.set(0);
    }

    public void periodic(){
    }


}
