package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase{
    TalonFX pivotMotor;
    CANrange canRange;
    PIDController pivotPID;
    CommandXboxController Xboxcontroller;
    TalonFXConfiguration PivotConfigs;
    TalonFXConfigurator PivotConfigurator;
    ProfiledPIDController ElevatorController;

    SparkMax IntakeMotor;
    SparkMaxConfig IntakeConfig;


   
    public GroundIntake (){
        
        pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID);
        PivotConfigs = new TalonFXConfiguration();
        PivotConfigurator = pivotMotor.getConfigurator();
        canRange = new CANrange(Constants.CANRANGE_ID);


        IntakeMotor = new SparkMax(Constants.GROUND_INTAKE_ID, MotorType.kBrushless);
        IntakeConfig = new SparkMaxConfig();
        IntakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40,40);
        IntakeMotor.configure(IntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        


        PivotConfigs.CurrentLimits.StatorCurrentLimit = 60;
        PivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        PivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(PivotConfigs);

        
    }

    public void pivotDown(){
        // for(double i = pivotMotor.getPosition().getValueAsDouble();  i < Constants.PIVOT_DOWN_POSITION+.5; ){
        
        if(pivotMotor.getPosition().getValueAsDouble() < Constants.PIVOT_DOWN_POSITION){
        pivotMotor.set(Constants.GROUND_PIVOT_SPEED);
        
        }
        else{
            pivotMotor.set(0);
        if (canRange.getIsDetected().getValue() == true) 
    {
        if(pivotMotor.getPosition().getValueAsDouble() <= Constants.PIVOT_UP_POSITION){
            IntakeMotor.set(-.3);
        }
        else{
        // Stop the motor if we have a note OR the pivot is too low
        IntakeMotor.set(0);
        }
    } else {
        // Run the motor only if it's safe AND we don't have a note
        IntakeMotor.set(-.3);
    }
}
        // i = pivotMotor.getPosition().getValueAsDouble();
        System.out.println("is pivot being called");
        
    // }
    double position = pivotMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Pivot Motor Rotation", position); 
        
    }

    public void pivotUp(){
        double position = pivotMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Pivot Motor Rotation", position); 
        System.out.println("pivotup isbeing claled");
        if(pivotMotor.getPosition().getValueAsDouble() > Constants.PIVOT_UP_POSITION){
        pivotMotor.set(-0.2);

        }
        if (canRange.getIsDetected().getValue() == true) 
    {
        if(pivotMotor.getPosition().getValueAsDouble() <= Constants.PIVOT_UP_POSITION){
            IntakeMotor.set(-.3);
            pivotMotor.set(0);
        }
        else{
        // Stop the motor if we have a note OR the pivot is too low
        IntakeMotor.set(0);
        }
    } else {
        // Run the motor only if it's safe AND we don't have a note
        IntakeMotor.set(-.3);
    }

    }
    public void IntakeOn(){
        if (canRange.getIsDetected().getValue() == true) 
    {
        if(pivotMotor.getPosition().getValueAsDouble() <= Constants.PIVOT_UP_POSITION){
            pivotMotor.set(0);
            IntakeMotor.set(-.3);
        }
        else{
        // Stop the motor if we have a note OR the pivot is too low
        IntakeMotor.set(0);
        }
    } else {
        // Run the motor only if it's safe AND we don't have a note
        IntakeMotor.set(-.3);
    }
    }

    public void IntakeReverse(){
        IntakeMotor.set(.3);
    }
    public void IntakeOff(){
        IntakeMotor.set(0);
    }
    
    public void off (){
        pivotMotor.set(0);
        IntakeMotor.set(0);
    }

    // MOVES PIVOT DOWN WITHOUT RUNNING INTAKE
public void pivotDownAuto() {
    if(pivotMotor.getPosition().getValueAsDouble() < Constants.PIVOT_DOWN_POSITION){
        pivotMotor.set(Constants.GROUND_PIVOT_SPEED);
    } else {
        pivotMotor.set(0);
    }
}

// MOVES PIVOT UP WITHOUT RUNNING INTAKE
public void pivotUpAuto() {
    if(pivotMotor.getPosition().getValueAsDouble() > Constants.PIVOT_UP_POSITION){
        pivotMotor.set(-0.2);
    } else {
        pivotMotor.set(0);
    }
}

public boolean hasNote() {
    // Returns TRUE if the sensor sees an object, FALSE otherwise
    return canRange.getIsDetected().getValue();
}

// Add this to GroundIntake.java
public double getPivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
}

    public void periodic(){
    }


}
