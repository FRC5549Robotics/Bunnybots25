// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

// import choreo.Choreo;
// import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveAuton;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Pivot;
//import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.DrivetrainSubsystem.direction;
// import frc.robot.subsystems.Elevator.PivotTarget;
//import frc.robot.commands.PivotAnalog;
//import frc.robot.commands.SnapBack;
//import frc.robot.subsystems.Pivot.PivotTarget;
//import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.DrivetrainSubsystem.direction;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.GroundIntake;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_controller = new CommandXboxController(Constants.DRIVE_CONTROLLER);
  private final CommandXboxController m_controller2 = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

  JoystickButton resetNavXButton = new JoystickButton(m_controller.getHID(), Constants.RESET_NAVX_BUTTON);

  JoystickButton shootHighButton = new JoystickButton(m_controller2.getHID(), 6);
  JoystickButton shootLowButton = new JoystickButton(m_controller2.getHID(), 5);
  JoystickButton intakeButton = new JoystickButton(m_controller2.getHID(), 11);
  JoystickButton groundIntakeButton = new JoystickButton(m_controller2.getHID(), 12);
  JoystickButton reverseIntakeButton = new JoystickButton(m_controller2.getHID(), 7);
  JoystickButton groundIntakeUpButton = new JoystickButton(m_controller2.getHID(), 1);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  //region Subsystems
  private final AHRS m_ahrs = new AHRS(NavXComType.kMXP_SPI);
  public final DrivetrainSubsystem m_drive = new DrivetrainSubsystem(m_ahrs);
  //private final Elevator m_elevator = new Elevator(m_controller2, setpointButtons);

  //private final Shintake m_shintake = new Shintake();
  // private final Climber m_climber = new Climber();
  private final Limelight m_limelight = new Limelight(m_drive, m_controller);
  private final Intake m_intake = new Intake();
  private final Shooter m_Shooter = new Shooter();
  private final Belt m_Belt = new Belt();
  private final GroundIntake m_pivot = new GroundIntake(m_controller2);

  // private final Climber m_climber = new Climber();
  //endregion

  //AUTOCHOOSER SET UP
  private final SendableChooser<Command> autoChooser;


  //region Trajectories
  

  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //region Drivetrain
     // m_controller.axisGreaterThan(0, 0.07).or(m_controller.axisGreaterThan(1, 0.07)).or(m_controller.axisGreaterThan(4, 0.07))
     // .or(m_controller.axisLessThan(0, -0.07)).or(m_controller.axisLessThan(1, -0.07)).or(m_controller.axisLessThan(4, -0.07))
     // .or(AutoAlignLeft).or(AutoAlignRight);
      //.onTrue(new DriveCommand(m_drive, m_controller, m_elevator, m_limelight));
      resetNavXButton.onTrue(new InstantCommand(m_drive::zeroGyroscope));
    //endregion
    
    //region Shooter, Intake, and Belt
    shootHighButton.onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootHigh), new InstantCommand(m_Belt::runBelt), new InstantCommand(m_intake::intake))).onFalse(Commands.parallel(new InstantCommand(m_Shooter::off), new InstantCommand(m_Belt::off), new InstantCommand(m_intake::off)));
  
    m_controller2.axisGreaterThan(Constants.SHOOT_HIGH, 0.7)
    .onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootHigh), new InstantCommand(m_Belt::runBelt)))
    .onFalse(Commands.parallel(new InstantCommand(m_Belt::off), new InstantCommand(m_Shooter::off)));
    m_controller2.axisGreaterThan(Constants.SHOOT_LOW, 0.7).onTrue(new InstantCommand(m_Shooter::shootLow)).onFalse(new InstantCommand(m_Shooter::off));
    m_controller2.axisGreaterThan(Constants.INTAKE, 0.7).onTrue(new InstantCommand(m_intake::intake)).onFalse(new InstantCommand(m_intake::off));
    reverseIntakeButton.onTrue(new InstantCommand(m_intake::reverse)).onFalse(new InstantCommand(m_intake::off));
    //groundIntakeButton.onTrue(new InstantCommand(m_))
    //shootHighButton.onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootHigh), new InstantCommand(m_Belt::runBelt), new InstantCommand(m_intake::intake))).onFalse(Commands.parallel(new InstantCommand(m_Shooter::off), new InstantCommand(m_Belt::off), new InstantCommand(m_intake::off)));
    //endregion

    



    //region Basic Testing Methods

    groundIntakeButton.onTrue(Commands.sequence(new InstantCommand(m_pivot::pivotDown), new InstantCommand(m_pivot::IntakeOn)))
    .onFalse(new InstantCommand(m_pivot::off));
    //endregion  
  }

  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Commands.sequence(new WaitCommand(0.25), resetOdometry, myTrajectory);
    // return new HardcodedAuton(m_drive, m_pivot, m_elevator, m_shintake);
    return autoChooser.getSelected();
  }
};
