package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    SparkMax IntakeMotor;
    SparkMaxConfig IntakeConfig;

    public Intake(){
        IntakeMotor = new SparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        IntakeConfig = new SparkMaxConfig();
        IntakeConfig.idleMode(IdleMode.kBrake);
        IntakeMotor.configure(IntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void intake(){
        System.out.println("Intake doesn't work! :(");
        IntakeMotor.set(Constants.INTAKE_SPEED);
        
    }

    public void reverse(){
        System.out.println("Intake aint workin");
        IntakeMotor.set(-Constants.INTAKE_SPEED);
    }
    public void off(){
        IntakeMotor.set(0);
        System.out.println("Intake Off");
    }

    
};
