package frc.robot.subsystems;

import java.io.Console;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final SparkMax m_turningController;
    private final SparkMax m_driveController;
    private SparkMaxConfig m_turningConfig, m_driveConfig;
  
    private final RelativeEncoder m_turningEncoder;
    private final RelativeEncoder m_driveEncoder;

    private final CANcoder m_turningCANCoder;

    // absolute offset for the CANCoder so that the wheels can be aligned when the
    // robot is turned on
//    private final Rotation2d m_CANCoderOffset;

    // private final SparkPIDController m_turningController;
    
    // private final SparkPIDController m_driveController;
    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     */
    public SwerveModule(
                        int driveMotorChannel,
                        int turningMotorChannel,
                        int turningCANCoderChannel,
                        double magnetOffset) {
        m_driveController = new SparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningController = new SparkMax(turningMotorChannel, MotorType.kBrushless);
        m_driveConfig = new SparkMaxConfig();
        m_turningConfig = new SparkMaxConfig();
        m_driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40, 40);
        m_turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40, 40);           
        m_driveConfig.encoder.velocityConversionFactor(Constants.kDriveConversionFactor / 60.0);
        m_driveConfig.encoder.positionConversionFactor(Constants.kDriveConversionFactor);
        m_turningConfig.encoder.positionConversionFactor(360.0 / Constants.kTurnPositionConversionFactor);
        
        m_driveConfig.closedLoop.pid(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        m_driveConfig.closedLoopRampRate(Constants.SLEW_RATE_LIMITER);
        m_turningConfig.closedLoop.pid(Constants.kTurningP, Constants.kTurningI, Constants.kTurningD);
        

        m_driveController.configure(m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turningController.configure(m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        // 401 only sets P of the drive PID

        new WaitCommand(0.5);
        m_driveEncoder = m_driveController.getEncoder();
        new WaitCommand(1);

        m_turningCANCoder = new CANcoder(turningCANCoderChannel);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.MagnetOffset = magnetOffset;
        m_turningCANCoder.setPosition(m_turningCANCoder.getAbsolutePosition().getValueAsDouble());
        m_turningCANCoder.getConfigurator().apply(config);
        // m_turningCANCoder.setPosition(0);        
        m_turningEncoder = m_turningController.getEncoder();
        // m_CANCoderOffset = Rotation2d.fromDegrees(turningCANCoderOffsetDegrees);

        // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
        // convert that to meters per second.
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // getPosition() returns the number of cumulative rotations.
        // Convert that to 0.0 to 1.0
        // double m1 = m_turningEncoder.getPosition() % 360.0;
        // double m2 = (m1 < 0) ? m1 + 360 : m1;

        double m2 = ((m_turningEncoder.getPosition()) % 360 + 360) % 360;
        String x;
        String y;
        x= null;
        y="3";
        String a = x != null ? y : x;

        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m2 * Math.PI / 180));
    }

    public SparkMax getTurnMotor() {
        return m_turningController;
    }

    public SparkMax getDriveMotor() {
        return m_driveController;
    }

    public RelativeEncoder getTurnEncoder() {
        return m_turningEncoder;
    }

    public RelativeEncoder getDriveEncoder() {
        return m_driveEncoder;
    }

    public CANcoder getTurnCANcoder() {
        return m_turningCANCoder;
    }

    public double getTurnCANcoderAngle() {
        return m_turningCANCoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public Rotation2d adjustedAngle = new Rotation2d();

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed (in meters per second?) and angle (in
     *              degrees).
     */
    public void setDesiredState(SwerveModuleState state) {

        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());

        if((driveOutput == 0 && Math.abs(getTurnEncoder().getPosition() - adjustedAngle.getDegrees()) > 2) || Math.abs(driveOutput) > 0){
            m_turningController.getClosedLoopController().setReference(
                adjustedAngle.getDegrees(),
                ControlType.kPosition
            );            
        }
        else{
            m_turningController.set(0);
        }

        SmartDashboard.putNumber("Commanded Velocity", driveOutput);
        SmartDashboard.putNumber("Module Speeds", m_driveEncoder.getVelocity());

        m_driveController.getClosedLoopController().setReference(driveOutput/Constants.kMaxSpeedMetersPerSecond, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0.0);
    }

    public void setOpenLoopState(SwerveModuleState state) {
        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());

        m_turningController.getClosedLoopController().setReference(
            adjustedAngle.getDegrees(),
            ControlType.kPosition
        );        

        SmartDashboard.putNumber("Commanded Velocity", driveOutput);

        m_driveController.setVoltage(Constants.kDriveFF * driveOutput);
    }

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    // Arguments are in radians.
    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {

        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistanceMeters() {
        return m_driveEncoder.getPosition();
    }

    public void resetDistance() {
        m_driveEncoder.setPosition(0.0);
    }

    public void syncTurningEncoders() {
        m_turningEncoder.setPosition(m_turningCANCoder.getAbsolutePosition().getValueAsDouble()*360);
    }

    public void switchToCoast() {
        m_driveConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40, 40);
        m_turningConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40, 40);
    }

    public void switchToBrake() {
        m_driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40, 40);
        m_turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40, 40);
    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
        // Reset the cumulative rotation counts of the SparkMax motors
    }
}
