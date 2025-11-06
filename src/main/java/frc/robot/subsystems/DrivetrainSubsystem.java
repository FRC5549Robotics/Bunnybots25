package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Optional;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import com.studica.frc.AHRS;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.commands.PPSwerveControllerCommand;
//import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import frc.robot.Constants;



@SuppressWarnings("PMD.ExcessiveImports")
public class DrivetrainSubsystem extends SubsystemBase {

  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
          Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
          Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
          Constants.FRONT_LEFT_MAGNET_OFFSET
          );

  private final SwerveModule m_frontRight =
      new SwerveModule(
          Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
          Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
          Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
          Constants.FRONT_RIGHT_MAGNET_OFFSET
          );

  private final SwerveModule m_rearLeft =
      new SwerveModule(
        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_ENCODER,
        Constants.BACK_LEFT_MAGNET_OFFSET
          );

  private final SwerveModule m_rearRight =
      new SwerveModule(
        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
        Constants.BACK_RIGHT_MAGNET_OFFSET
          );

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
  private double lastTime;
  private final Timer timer;

  // The gyro sensor
  AHRS m_ahrs;
//  private final Gyro m_gyro =  new ADIS16470_IMU(); // new ADXRS450_Gyro();
  // private final PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.kPigeonPort);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  //target pose and controller
  Pose2d m_targetPose;
  PIDController m_thetaController = new PIDController(0.1, 0.0, 0.001);
  PIDController m_translationController = new PIDController(0.1,0,0);

  ChassisSpeeds speeds; 
  Field2d m_field;

  //Config
  RobotConfig config;
    
  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem(AHRS ahrs) {
    m_ahrs = ahrs;

    // Zero out the gyro
    m_odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), getModulePositions());

    // for (SwerveModule module: modules) {
    //   module.resetDistance();
    //   module.syncTurningEncoders();
    // }

    for (int i = 0; i < 4; i++) {
      modules[i].resetDistance();
      modules[i].syncTurningEncoders();
      
      new WaitCommand(1);
      // System.out.println("Module" + i + "is synced to" + modules[i].getTurnCANcoderAngle() + "  " + modules[i].getTurnEncoder().getPosition());
    }

    m_targetPose = m_odometry.getPoseMeters();
    m_thetaController.reset();
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    timer = new Timer();
    timer.reset();
    timer.start();
    lastTime = 0;

    //PATHPLANNER CONFIG

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e){
      e.printStackTrace();
    }  

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.05, 0.0, 2.0), // Translation PID constants
                    new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration                           cs
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }

  public void syncEncoders() {
    // for (SwerveModule module: modules) {
    //   module.resetDistance();
    //   module.syncTurningEncoders();
    // }

    for (int i = 0; i < 4; i++) {
      modules[i].resetDistance();
      modules[i].syncTurningEncoders();
    }
  }
  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry(); 

    SmartDashboard.putNumber("Front Left CANCoder", m_frontLeft.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Front Right CANCoder", m_frontRight.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Back Left CANCoder", m_rearLeft.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Back Right CANCoder", m_rearRight.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);

    SmartDashboard.putNumber("Front Left Neo Encoder", m_frontLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Front Right Neo Encoder", m_frontRight.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Back Left Neo Encoder", m_rearLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Back Right Neo Encoder", m_rearRight.getTurnEncoder().getPosition());
    
    SmartDashboard.putNumber("Front Left Neo Velocity", m_frontLeft.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Front Right Neo Velocity", m_frontRight.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Back Left Neo Velocity", m_rearLeft.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Back Right Neo Velocity", m_rearRight.getDriveEncoder().getVelocity());

    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    
    SmartDashboard.putNumber("currentX", getPose().getX());
    SmartDashboard.putNumber("currentY", getPose().getY());
    SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());
    SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());
    
    // This was due to pinion slippage: If it is still happening, uncomment this code
    // if(Math.abs(m_frontRight.getTurnEncoder().getPosition() - m_frontRight.getTurnCANcoderAngle()) > 2){
    //   m_frontRight.getTurnEncoder().setPosition(m_frontRight.getTurnCANcoderAngle());
    // }

    // SmartDashboard.putNumber("Distance 0", modules[0].getDriveDistanceMeters());
    // SmartDashboard.putNumber("Distance 1", modules[1].getDriveDistanceMeters());
    // SmartDashboard.putNumber("Distance 2", modules[2].getDriveDistanceMeters());
    // SmartDashboard.putNumber("Distance 3", modules[3].getDriveDistanceMeters());

    // SmartDashboard.putNumber("Angle 0", modules[0].getTurnCANcoderAngle());
    // SmartDashboard.putNumber("Angle 1", modules[1].getTurnCANcoderAngle());
    // SmartDashboard.putNumber("Angle 2", modules[2].getTurnCANcoderAngle());
    // SmartDashboard.putNumber("Angle 3", modules[3].getTurnCANcoderAngle());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    // if (1 <= timer.get() && timer.get() <= 1.5) {
    //   m_ahrs.zeroYaw();
      // System.out.println("Zeroed: " + getHeading());
  }
  

  public void updateOdometry() {
    double time = timer.get();
    double dt = time - lastTime;
    lastTime = time;
    if (dt == 0) return;
    m_odometry.update(getHeading(), getModulePositions());
  }


  public double getTranslationalVelocity() {
    return Math.hypot(this.speeds.vxMetersPerSecond, this.speeds.vyMetersPerSecond);
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // public void setAutonPose(PathPlannerPath traj) {
  //   m_field.setRobotPose(traj.getPreviewStartingHolonomicPose());
  // }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    /* Don't reset all the motors' positions. Otherwise the robot thinks it has teleported!
    for (SwerveModule module: modules) {
      module.resetDistance();
    }
    */
    m_odometry.resetPosition(
      getHeading(), 
      getModulePositions(), 
      pose);
  }
  public void resetOdometry() {
    /* Don't reset all the motors' positions. Otherwise the robot thinks it has teleported!
    for (SwerveModule module: modules) {
      module.resetDistance();
    }
    */
    m_odometry.resetPosition(
      getHeading(), 
      getModulePositions(), 
      new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Method to rotate the relative orientation of the target pose at a given rate.
   *    
   * @param deltaTheta How much to rotate the target orientation per loop.
   */
  public void rotateRelative(Rotation2d deltaTheta) {
    Transform2d transform = new Transform2d(new Translation2d(), deltaTheta);
    m_targetPose = m_targetPose.transformBy(transform);
  }

  /**
   * Method to set the absolute orientation of the target pose.
   *
   * @param theta The target orientation.
   */
  public void rotateAbsolute(Rotation2d theta) {
    m_targetPose = new Pose2d(new Translation2d(), theta);
  }

  /**
   * Method to get the output of the chassis orientation PID controller.
   *
   */
  public double getThetaDot() {
    double setpoint = m_targetPose.getRotation().getRadians();
    double measurement = getPose().getRotation().getRadians();
    double output = m_thetaController.calculate(measurement, setpoint);
    SmartDashboard.putNumber("PID out", output);
    return output;
  }

  /**
   * Method to drive the robot with given velocities.
   *
   * @param speeds ChassisSpeeds object with the desired chassis speeds [m/s and rad/s].
   */
  @SuppressWarnings("ParameterName")
  public void drive(ChassisSpeeds speeds, boolean normalize) {

    this.speeds = speeds;

    if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
      snap();
      return;
    }

    SwerveModuleState[] swerveModuleStates =
        Constants.kDriveKinematics.toSwerveModuleStates(speeds);

    if (normalize) normalizeDrive(swerveModuleStates, speeds);
    
    // System.out.println(swerveModuleStates[0]+":"+swerveModuleStates[1]+":"+swerveModuleStates[2]+":"+swerveModuleStates[3]);
    setModuleStates(swerveModuleStates);
  }

  public void openLoopDrive(ChassisSpeeds speeds) {
    this.speeds = speeds;
    if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
      snap();
      return;
    }

    SwerveModuleState[] swerveModuleStates =
        Constants.kDriveKinematics.toSwerveModuleStates(speeds);
           
    normalizeDrive(swerveModuleStates, speeds);
    
    setModuleStates(swerveModuleStates);
  }

  public void AutonDrive() {
    drive(new ChassisSpeeds(-0.5, 0, 0), true);
  }

  public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
    double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / Constants.kMaxTranslationalVelocity;
    double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / Constants.kMaxRotationalVelocity;
    double k = Math.max(translationalK, rotationalK);

    // Find the how fast the fastest spinning drive motor is spinning                                       
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : desiredStates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      SmartDashboard.putNumber("Capped Speed", realMaxSpeed);
    }

    double scale = Math.min(k * Constants.kMaxTranslationalVelocity / realMaxSpeed, 1);
    for (SwerveModuleState moduleState : desiredStates) {
      moduleState.speedMetersPerSecond *= scale;
    }
  }

  public void snap() {
    for (SwerveModule module : modules) {
      module.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Preferences.getDouble("kMaxSpeedMetersPerSecond", Constants.kMaxSpeedMetersPerSecond));

        for (int i = 0; i <= 3; i++) {
          modules[i].setDesiredState(desiredStates[i]);
        }
  }

  public void setOpenLoopStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Preferences.getDouble("kMaxSpeedMetersPerSecond", Constants.kMaxSpeedMetersPerSecond));

    for (int i = 0; i <= 3; i++) {
      modules[i].setOpenLoopState(desiredStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
      
    }

    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    for (SwerveModule module: modules) {
      module.resetEncoders();
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_ahrs.zeroYaw();
    // offset = 0;
    m_targetPose = new Pose2d(new Translation2d(), new Rotation2d());
  }

  public void resetOdometry(double heading, Pose2d pose) {
    zeroHeading();
    // offset = heading;
    m_odometry.resetPosition(Rotation2d.fromDegrees(heading),
    getModulePositions(),
    pose);
  }
  public void zeroGyroscope(){
    m_ahrs.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    double raw_yaw = -m_ahrs.getYaw();
    // double raw_yaw = m_ahrs.getYaw();
    SmartDashboard.putNumber("Raw Yaw", raw_yaw);
    // float raw_yaw = m_ahrs.getYaw(); // Returns yaw as -180 to +180.
    double calc_yaw = raw_yaw;

    if (0.0 > raw_yaw ) { // yaw is negative
      calc_yaw += 360.0;
    }
    return Rotation2d.fromDegrees(calc_yaw);
  }

  

  private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      new SwerveModulePosition(-m_frontLeft.getDriveDistanceMeters(), m_frontLeft.getState().angle),
      new SwerveModulePosition(-m_frontRight.getDriveDistanceMeters(), m_frontRight.getState().angle),
      new SwerveModulePosition(-m_rearLeft.getDriveDistanceMeters(), m_rearLeft.getState().angle),
      new SwerveModulePosition(-m_rearRight.getDriveDistanceMeters(), m_rearRight.getState().angle)};
  }

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] states = Constants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  //AUTOBUILDER
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }
  

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();
    System.out.println(sample + "");
    System.out.println(sample.vx + ":" + sample.vy + ":" + sample.omega);
    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
        sample.vx - m_translationController.calculate(pose.getX(), sample.x),
        sample.vy - m_translationController.calculate(pose.getY(), sample.y),
        sample.omega + m_thetaController.calculate(pose.getRotation().getRadians(), sample.heading)
    );

    // Apply the generated speeds
    drive(speeds, true);
  }
  
  // public void snapWheels(){
  //   for (SwerveModule swerveModule : modules) {
  //     double positionMod = ((swerveModule.getTurnEncoder().getPosition() % 360) + 360) % 360;
  //     swerveModule.getTurnMotor().getClosedLoopController().setReference(0, ControlType.kPosition);
  //   }
  // }

  //Not so gourmet code
  public static List<Double> generateSpeeds(boolean[] bool_values, double[] controllerVals) {

    double xDot = 0;
    double yDot = 0;
    double thetaDot =0;

    List<Double> mutableVals = new ArrayList<>();
    mutableVals.add(xDot);
    mutableVals.add(yDot);
    mutableVals.add(thetaDot);


    for (int i = 0; i < 3; i ++) {
      if (!bool_values[i]) {
        mutableVals.set(i, null);
        continue;
      }
      
      double Dot;
      if(controllerVals[i] < 0){
        Dot = -(controllerVals[i]*controllerVals[i]*Constants.kMaxTranslationalVelocity);
      }
      else{
        Dot = (controllerVals[i]*controllerVals[i]) * Constants.kMaxTranslationalVelocity;
      }

      if(Math.abs(Dot)<=0.07*0.07*Constants.kMaxTranslationalVelocity){
        Dot = 0;
      }
      else{
        if(Dot > 0){
          Dot -= (0.07*0.07);
        }
        else{
          Dot += (0.07*0.07);
        }
        Dot *= 1/(1-(0.07*0.07));
      }
      mutableVals.set(i, Dot);

    }

    return mutableVals;
  }

  //SET INVERTED THE STUPID FREAKING BACK LEFT DRIVE

  public void switchToCoast(){
    m_frontLeft.switchToCoast();
    m_frontRight.switchToCoast();
    m_rearLeft.switchToCoast();
    m_rearRight.switchToCoast();
  }

  public void switchToBrake(){
    m_frontLeft.switchToBrake();
    m_frontRight.switchToBrake();
    m_rearLeft.switchToBrake();
    m_rearRight.switchToBrake();
  }
  //Gourmet code
  public double[] processControllerInputs(double[] ControllerSpeeds){
    ArrayList<Double> controllerSpeeds= new ArrayList<>();
    ArrayList<Double> adjustedSpeeds = new ArrayList<>();
    Iterator<Double> speedsIterator = controllerSpeeds.iterator();
    for (double d : ControllerSpeeds) {
      controllerSpeeds.add(d);
    }
    while(true){
      if(speedsIterator.hasNext()){
        Double speed = speedsIterator.next();
        if(speed < 0){
          speed = -(speed * speed * Constants.kMaxTranslationalVelocity);
        }
        else{
          speed = speed * speed * Constants.kMaxTranslationalVelocity;
        }
        if(Math.abs(speed)<=0.07*0.07*Constants.kMaxTranslationalVelocity){
          speed = 0.0;
        }
        else{
          if(speed > 0){
            speed -= (0.07*0.07);
          }
          else{
            speed += (0.07*0.07);
          }
          speed *= 1/(1-(0.07*0.07));
        }
        adjustedSpeeds.add(speed);
      }
      else{
        break;
      }
    }
    return adjustedSpeeds.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public void off() {
    drive(new ChassisSpeeds(0, 0, 0), false);
    System.out.println("Drive turned off.");
  }
  
  public enum direction{
    left,
    right,
    forward,
    backward,
  }
}