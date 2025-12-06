package frc.robot.subsystems;

import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase {
    private final CANdle candle = new CANdle(Constants.CANDLE_ID);
    // private final Limelight limelight = new Limelight();
    private final FireAnimation fire = new FireAnimation(0, 20)
    .withSlot(0).withBrightness(1.0).withSparking(.5).withCooling(.3).withFrameRate(20).withDirection(AnimationDirectionValue.Forward);


    public Candle() {
        
    }

    public void flameOn() {
        System.out.println("Candle is being called");
        candle.setControl(fire);
    }

}