package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    CANdle candle;
    CANrange canRange;
    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx   = 37;

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx   = 67;

    RGBWColor kGreen = new RGBWColor(0,217,0 ,0);
    RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(.5);
    RGBWColor kViolet =  RGBWColor.fromHSV(Degrees.of(270), .9, .8);
    RGBWColor kRed =  RGBWColor.fromHex("#D9000000").orElseThrow();
    //probably gonna mainly use hexcode or rgb values but refrence ig.

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<>();

    public LEDSubsystem(){
        candle = new CANdle(Constants.CANDLE_ID);
        canRange = new CANrange(Constants.CANRANGE_ID);

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = .5;
        // idk if the type is GRB

        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(config);

        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }

        candle.setControl(new SolidColor(0, 3).withColor(kGreen));  // first 4
        candle.setControl(new SolidColor(4, 7).withColor(kWhite));  // next 4

        // Slot 0 chooser (first segment of strip)
        m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Rainbow",    AnimationType.Rainbow);
        m_anim0Chooser.addOption("Twinkle",    AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off",AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire",       AnimationType.Fire);

        // Slot 1 chooser (second segment of strip)
        m_anim1Chooser.setDefaultOption("Larson",     AnimationType.Larson);
        m_anim1Chooser.addOption("RGB Fade",          AnimationType.RgbFade);
        m_anim1Chooser.addOption("Single Fade",       AnimationType.SingleFade);
        m_anim1Chooser.addOption("Strobe",            AnimationType.Strobe);
        m_anim1Chooser.addOption("Fire",              AnimationType.Fire);

        SmartDashboard.putData("Animation 0", m_anim0Chooser);
        SmartDashboard.putData("Animation 1", m_anim1Chooser);

    }
    public void setIntaking(){
        if (canRange.getIsDetected().getValue() == false) {
        candle.setControl(
            new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(kGreen)
        );
        }
        else{
            candle.setControl(new SolidColor(0, 67).withColor(kGreen));
        }

    }


    public void setIdleRainbow(){
        candle.setControl(
            new RainbowAnimation(0, 67).withSlot(0));
    }

    public void periodic (){
        final var anim0Selection = m_anim0Chooser.getSelected();
        if (m_anim0State != anim0Selection && anim0Selection != null) {
            m_anim0State = anim0Selection;

            switch (m_anim0State) {
                default:
                case ColorFlow:
                    candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx)
                            .withSlot(0)
                            .withColor(kViolet)
                    );
                    break;

                case Rainbow:
                    candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx)
                            .withSlot(0)
                    );
                    break;

                case Twinkle:
                    candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx)
                            .withSlot(0)
                            .withColor(kViolet)
                    );
                    break;

                case TwinkleOff:
                    candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx)
                            .withSlot(0)
                            .withColor(kViolet)
                    );
                    break;

                case Fire:
                    candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx)
                            .withSlot(0)
                    );
                    break;
            }
        }
    
    final var anim1Selection = m_anim1Chooser.getSelected();
        if (m_anim1State != anim1Selection && anim1Selection != null) {
            m_anim1State = anim1Selection;

            switch (m_anim1State) {
                default:
                case Larson:
                    candle.setControl(
                        new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx)
                            .withSlot(1)
                            .withColor(kRed)
                    );
                    break;

                case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                            .withSlot(1)
                    );
                    break;

                case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                            .withSlot(1)
                            .withColor(kRed)
                    );
                    break;

                case Strobe:
                    candle.setControl(
                        new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                            .withSlot(1)
                            .withColor(kRed)
                    );
                    break;

                case Fire:
                    candle.setControl(
                        new FireAnimation(kSlot1StartIdx, kSlot1EndIdx)
                            .withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
            }
        }
    }
}


