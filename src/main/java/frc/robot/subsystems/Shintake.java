package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shintake extends SubsystemBase{
    //create the two neo motors

   SparkMax IntakeMotorLeft;
   SparkMax IntakeMotorRight;
   SparkMaxConfig IntakeLeftConfig, IntakeRightConfig;

   public Shintake(){
        IntakeMotorLeft = new SparkMax(Constants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
        IntakeMotorRight = new SparkMax(Constants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);
        IntakeLeftConfig = new SparkMaxConfig();
        IntakeRightConfig = new SparkMaxConfig();
        IntakeLeftConfig.idleMode(IdleMode.kCoast);
        IntakeRightConfig.idleMode(IdleMode.kCoast);

        IntakeRightConfig.smartCurrentLimit(20);// changed to 20 from 40 -Jordan
        IntakeLeftConfig.smartCurrentLimit(20); // this too


        IntakeMotorLeft.configure(IntakeLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        IntakeMotorRight.configure(IntakeRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    // public void intake(double speed) {//make dependent on trigger
    //     IntakeMotorLeft.set(-speed);
    //     IntakeMotorRight.set(-speed);   
    //     System.out.println("intake doesn't fucking work");
    // }
      
    // public void alt_intake() {//make dependent on trigger
    //     IntakeMotorLeft.set(-0.7);
    //     IntakeMotorRight.set(-0.7);
    // }

    public void shoot() {
      IntakeMotorLeft.set(-Constants.INTAKE_OUTTAKE_SPEED);
      IntakeMotorRight.set(Constants.INTAKE_OUTTAKE_SPEED);
      System.out.println("shoot pls work");
      System.out.println(Constants.INTAKE_OUTTAKE_SPEED);
    }
    public void shootL1(){
      IntakeMotorLeft.set(-Constants.OUTTAKE_L1_SPEED);
      IntakeMotorRight.set(Constants.OUTTAKE_L1_SPEED*0.4);
    }
    public void intake() {
      IntakeMotorLeft.set(Constants.INTAKE_OUTTAKE_SPEED);
      IntakeMotorRight.set(-Constants.INTAKE_OUTTAKE_SPEED);
      System.out.println("intake doesn't work");
      System.out.println(Constants.INTAKE_OUTTAKE_SPEED);

    }
    public void off(){
        IntakeMotorLeft.set(0);
        IntakeMotorRight.set(0);
        System.out.println("off");
      }
}
    

