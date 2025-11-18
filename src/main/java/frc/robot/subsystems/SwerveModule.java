package frc.robot.subsystems;

import java.io.Console;

import org.opencv.core.Mat;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
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

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final TalonFX m_turningController;
    private final TalonFX m_driveController;
    private TalonFXConfiguration m_turningConfig, m_driveConfig;
    private final CANcoder m_turningCANcoder;
    private final CANcoder m_driveCANcoder;
    

  
    // private final RelativeEncoder m_turningEncoder;
    //private final RelativeEncoder m_driveEncoder;


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

    public SwerveModule( int driveMotorChannel, int turningMotorChannel, int turningCANCoderChannel, double magnetOffset, CANBus canbus) {
        //Instantiate SwerveModule components
        m_driveController = new TalonFX(driveMotorChannel, canbus);
        m_turningController = new TalonFX(turningMotorChannel, canbus);
        m_turningCANcoder = new CANcoder(turningCANCoderChannel, canbus);
        m_driveCANcoder = new CANcoder(driveMotorChannel, canbus);

        //Configure SwerveModule components
        m_driveConfig = new TalonFXConfiguration();
        m_turningConfig = new TalonFXConfiguration();

        // --- Drive Config ---
        m_driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        m_driveConfig.CurrentLimits.withStatorCurrentLimit(40).withSupplyCurrentLimit(40);

        // --- Turning Config ---
        m_turningConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        m_turningConfig.CurrentLimits.withStatorCurrentLimit(40).withSupplyCurrentLimit(40);

        // --- !! APPLY CONFIGS !! ---
        m_driveController.getConfigurator().apply(m_driveConfig);
        m_turningController.getConfigurator().apply(m_turningConfig);
        
        //m_driveConfig.encoder.velocityConversionFactor(Constants.kDriveConversionFactor / 60.0);
        // var m_driveConfig = m_driveController.getVelocity().getValue();
        // var m_turningConfig = m_turningController.getPosition().getValue();
        // var m_turingDriveConfig = m_driveController.getPosition().getValue();

        //m_driveConfig.encoder.positionConversionFactor(Constants.kDriveConversionFactor);
        //m_turningConfig.encoder.positionConversionFactor(360.0 / Constants.kTurnPositionConversionFactor);
        
        // m_driveConfig.closedLoop.pid(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        
        // --- Drive PID ---
        var pidDriveConfigs = new Slot0Configs();
        pidDriveConfigs.kP = Constants.kDriveP; 
        pidDriveConfigs.kI = Constants.kDriveI;
        pidDriveConfigs.kD = Constants.kDriveD; 
        m_driveController.getConfigurator().apply(pidDriveConfigs);

        //DO WHAT I DID FOR TURNING FOR DRIVE GUYS!!!
        //m_driveController.closedLoopRampRate(Constants.SLEW_RATE_LIMITER);
        
        //m_turningConfig.closedLoop.pid(Constants.kTurningP, Constants.kTurningI, Constants.kTurningD);
        // --- Turning PID ---
        var pidTurningConfigs = new Slot0Configs();
        pidTurningConfigs.kP = Constants.kTurningP;
        pidTurningConfigs.kI = Constants.kTurningI; 
        pidTurningConfigs.kD = Constants.kTurningD;
        m_turningController.getConfigurator().apply(pidTurningConfigs);


        // m_driveController.configure(m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // m_turningController.configure(m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        // 401 only sets P of the drive PID

        //new WaitCommand(0.5);
        //m_driveEncoder = m_driveController.getConfigurator();
        //new WaitCommand(1);

        // --- CANcoder Config ---
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = magnetOffset;
        m_turningCANcoder.getConfigurator().apply(config);

        //Connect CaNCoder to Turning Controller
        // Wait for CANcoder to get a valid signal
        Timer.delay(0.5); 
        // Get absolute position (in rotations 0.0-1.0)
        double absolutePosition = m_turningCANcoder.getAbsolutePosition().getValueAsDouble();
        // Tell the TalonFX its position (in rotations)
        m_turningController.setPosition(absolutePosition);

        // m_turningencoder.setPosition(m_turningCANcoder.getAbsolutePosition().getValueAsDouble());
        // m_turningCANCoder.setPosition(0);        
        // TalonFXConfigurator m_turningEncoder = m_turningController.getConfigurator(); //aAWHAWFHAWDHAWJHFIAHWDNAWFJLKJjAGLk
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

        // 1. Get Drive Velocity
        // getVelocity() returns Rotations Per Second (RPS) of the motor shaft
        double driveRPS = m_driveController.getVelocity().getValueAsDouble();
        // You MUST convert RPS to meters per second using your gear ratio and wheel size
        // You'll need to re-calculate and define kDriveConversionFactor in your Constants file
        double driveMetersPerSecond = driveRPS * Constants.kDriveConversionFactor;

        // 2. Get Turning Angle
        // getAbsolutePosition() returns rotations (0.0 to 1.0)
        double turningRotations = m_turningCANcoder.getAbsolutePosition().getValueAsDouble();
        // Convert rotations to degrees for the Rotation2d object
        double turningDegrees = turningRotations * 360.0;

        // The line you mentioned (m2) is no longer needed.
        // String x;
        // String y;
        // x = null;
        // y = "3";
        // String a = x != null ? y : x; // This code doesn't do anything, you can remove it.
        return new SwerveModuleState(driveMetersPerSecond, Rotation2d.fromDegrees(turningDegrees));
    }
    

    public TalonFX getTurnMotor() {
        return m_turningController;
    }

    public TalonFX getDriveMotor() {
        return m_driveController;
    }

    // public CANcoder getTurnEncoder() {
    //     return m_turningEncoder;
    // }

    public CANcoder getDriveCANcoder() {
        return m_driveCANcoder;
    }

    public CANcoder getTurnCANcoder() {
        return m_turningCANcoder;
    }

    public double getTurnCANcoderAngle() {
        return m_turningCANcoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public Rotation2d adjustedAngle = new Rotation2d();

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed (in meters per second?) and angle (in
     *              degrees).
     */
    public void setDesiredState(SwerveModuleState state) {
        // !! FIX: Use getAbsolutePosition() !!
        // getPosition() is cumulative, getAbsolutePosition() is 0.0-1.0
        double curRotations = m_turningCANcoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d curAngle = Rotation2d.fromDegrees(curRotations * 360.0);

        // Rotation2d curAngle = Rotation2d.fromDegrees(m_turningCANcoder.getPosition().getValueAsDouble());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());
        double targetRotations = adjustedAngle.getRotations();
        PositionVoltage positionRequest = new PositionVoltage(targetRotations);
        

        if((driveOutput == 0 && Math.abs(getTurnCANcoder().getPosition().getValueAsDouble() - adjustedAngle.getDegrees()) > 2) || Math.abs(driveOutput) > 0){
            // m_turningController.getClosedLoopController().setReference(
            //     adjustedAngle.getDegrees(),
            //     ControlType.kPosition
            // );     
            m_turningController.setControl(positionRequest);       
        }
        else{
            m_turningController.set(0);
        }

        SmartDashboard.putNumber("Commanded Velocity", driveOutput);
        SmartDashboard.putNumber("Module Speeds", m_driveCANcoder.getVelocity().getValueAsDouble());

        // m_driveController.getClosedLoopController().setReference(driveOutput/Constants.kMaxSpeedMetersPerSecond, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0.0);
        DutyCycleOut driveRequest = new DutyCycleOut(driveOutput / Constants.kMaxSpeedMetersPerSecond);
        m_driveController.setControl(driveRequest);
    }

    public void setOpenLoopState(SwerveModuleState state) {
        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningCANcoder.getPosition().getValueAsDouble());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        // --- Turning Control ---
        PositionVoltage turnRequest = new PositionVoltage(adjustedAngle.getRotations());
        m_turningController.setControl(turnRequest);

        // --- Drive Control ---
        VoltageOut driveRequest = new VoltageOut(Constants.kDriveFF * driveOutput);
        m_driveController.setControl(driveRequest);

        SmartDashboard.putNumber("Commanded Velocity", driveOutput);

        // m_driveController.setVoltage(Constants.kDriveFF * driveOutput);
    }

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    // Arguments are in radians.
    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {

        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistanceMeters() {
        return m_driveCANcoder.getPosition().getValueAsDouble();
    }

    public void resetDistance() {
        m_driveCANcoder.setPosition(0.0);
    }

    // public void syncTurningEncoders() {
    //     m_turningEncoder.setPosition(m_turningCANCoder.getAbsolutePosition().getValueAsDouble()*360);
    // }

    public void switchToCoast() {
        m_driveController.setNeutralMode(NeutralModeValue.Coast);
        m_turningController.setNeutralMode(NeutralModeValue.Coast);
    }
    
    public void switchToBrake() {
        m_driveController.setNeutralMode(NeutralModeValue.Brake);
        m_turningController.setNeutralMode(NeutralModeValue.Brake);
    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
        // Reset the cumulative rotation counts of the SparkMax motors
    }
}
