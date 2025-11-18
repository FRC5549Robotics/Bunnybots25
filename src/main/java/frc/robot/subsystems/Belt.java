package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Belt extends SubsystemBase {
    private SparkMax BeltMotor;
    private SparkMaxConfig BeltConfig;

    public Belt() {
        BeltMotor = new SparkMax(Constants.BELT_MOTOR_ID, MotorType.kBrushless);

        // Configure the motor
        BeltConfig = new SparkMaxConfig();
        BeltConfig.idleMode(IdleMode.kCoast); // Let it spin freely when not powered
        // BeltMotor.applyConfig(BeltConfig);
    }

    /** Runs the belt motor at 50% power (forward) */
    public void runBelt() {
        BeltMotor.set(0.5);  // Change 0.5 to any value from -1.0 to 1.0 as needed
    }

    /** Stops the belt motor */
    public void off() {
        BeltMotor.set(0.0);
    }
}
