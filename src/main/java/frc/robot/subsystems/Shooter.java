package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    SparkMax OuttakeMotorLeft;
    SparkMax OuttakeMotorRight;
    SparkMaxConfig OuttakeMotorLeftConfig;
    SparkMaxConfig OuttakeMotorRightConfig;
    public Shooter(){
        OuttakeMotorLeft = new SparkMax(Constants.OUTTAKE_MOTOR_LEFT, MotorType.kBrushless);
        OuttakeMotorLeftConfig = new SparkMaxConfig();
        OuttakeMotorLeftConfig.idleMode(IdleMode.kBrake);
        OuttakeMotorLeft.configure(OuttakeMotorLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        OuttakeMotorRight = new SparkMax(Constants.OUTTAKE_MOTOR_RIGHT, MotorType.kBrushless);
        OuttakeMotorRightConfig = new SparkMaxConfig();
        OuttakeMotorRightConfig.idleMode(IdleMode.kBrake);
        OuttakeMotorRight.configure(OuttakeMotorRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
