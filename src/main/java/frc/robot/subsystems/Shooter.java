package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    TalonFX OuttakeMotorLeft;
    TalonFX OuttakeMotorRight;
    TalonFXConfiguration OuttakeMotorLeftConfig;
    TalonFXConfiguration OuttakeMotorRightConfig;

    
    public Shooter(){
        OuttakeMotorLeft = new TalonFX(Constants.OUTTAKE_MOTOR_LEFT);
        OuttakeMotorLeftConfig = new TalonFXConfiguration();
        OuttakeMotorLeftConfig.CurrentLimits.StatorCurrentLimit = 60;
        OuttakeMotorLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        OuttakeMotorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        OuttakeMotorRight = new TalonFX(Constants.OUTTAKE_MOTOR_RIGHT);
        OuttakeMotorRightConfig = new TalonFXConfiguration();
        OuttakeMotorRightConfig.CurrentLimits.StatorCurrentLimit = 60;
        OuttakeMotorRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        OuttakeMotorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        OuttakeMotorRight.getConfigurator().apply(OuttakeMotorRightConfig);
        OuttakeMotorLeft.getConfigurator().apply(OuttakeMotorLeftConfig);
    }

    public void shootHigh(){
        OuttakeMotorLeft.set(Constants.OUTTAKE_MOTOR_LEFT_SPEED_HIGH);
        OuttakeMotorRight.set(Constants.OUTTAKE_MOTOR_RIGHT_SPEED_HIGH);
        // System.out.println("Intake doesn't work! :(");
        System.out.println(Constants.OUTTAKE_MOTOR_LEFT_SPEED_HIGH);
        System.out.println(Constants.OUTTAKE_MOTOR_RIGHT_SPEED_HIGH);
    }

    public void shootLow(){
        OuttakeMotorLeft.set(Constants.OUTTAKE_MOTOR_LEFT_SPEED_LOW);
        OuttakeMotorRight.set(Constants.OUTTAKE_MOTOR_RIGHT_SPEED_LOW);
        // System.out.println("Intake doesn't work! :(");
        System.out.println(Constants.OUTTAKE_MOTOR_LEFT_SPEED_LOW);
        System.out.println(Constants.OUTTAKE_MOTOR_RIGHT_SPEED_LOW);
    }

    public void off(){
        OuttakeMotorLeft.set(0);
        OuttakeMotorRight.set(0);
        System.out.println("Outtake Off");
    }
};
