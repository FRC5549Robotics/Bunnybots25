// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.DrivetrainSubsystem.direction;
// import frc.robot.subsystems.Elevator.PivotTarget;


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

  JoystickButton stowedButton = new JoystickButton(m_controller2.getHID(), 1);
  JoystickButton pivotIntakeButton = new JoystickButton(m_controller2.getHID(), 4);
  JoystickButton L1Button = new JoystickButton(m_controller2.getHID(), 3);
  JoystickButton L2Button = new JoystickButton(m_controller2.getHID(), 5);
  JoystickButton L3Button = new JoystickButton(m_controller2.getHID(), 6);
  // JoystickButton L4Button = new JoystickButton(m_controller2.getHID(), 0);
  POVButton AlgaeLowButton = new POVButton(m_controller2.getHID(), 180);
  POVButton AlgaeHighButton = new POVButton(m_controller2.getHID(), 0);
  POVButton ProcessorButton = new POVButton(m_controller2.getHID(), 90);
  POVButton climbSetpointButton = new POVButton(m_controller2.getHID(), 270);
  JoystickButton climbButton = new JoystickButton(m_controller2.getHID(), Constants.CLIMB_BUTTON);
  JoystickButton climbUnwind = new JoystickButton(m_controller2.getHID(), Constants.CLIMB_UNWIND);
  JoystickButton l1EjectButton = new JoystickButton(m_controller2.getHID(), 2);
  JoystickButton AutoAlignLeft = new JoystickButton(m_controller.getHID(), 3);
  JoystickButton AutoAlignRight = new JoystickButton(m_controller.getHID(), 2);
  Trigger[] setpointButtons = {stowedButton, pivotIntakeButton, L1Button, L2Button, L3Button, AlgaeHighButton, AlgaeLowButton};

  //DANCE BUTTONS
  JoystickButton slideRight = new JoystickButton(m_controller.getHID(), 2);
  JoystickButton slideLeft = new JoystickButton(m_controller.getHID(), 3);
  JoystickButton slideForward = new JoystickButton(m_controller.getHID(), 4);
  JoystickButton slideBackward = new JoystickButton(m_controller.getHID(), 1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  //region Subsystems
  private final AHRS m_ahrs = new AHRS(NavXComType.kMXP_SPI);
  public final DrivetrainSubsystem m_drive = new DrivetrainSubsystem(m_ahrs);
  //private final Elevator m_elevator = new Elevator(m_controller2, setpointButtons);
  //private final Pivot m_pivot = new Pivot(m_controller2, setpointButtons);
  private final Shintake m_shintake = new Shintake();
  // private final Climber m_climber = new Climber();
  private final Limelight m_limelight = new Limelight(m_drive, m_controller);

  // private final Climber m_climber = new Climber();
  //endregion

  //AUTOCHOOSER SET UP
  private final SendableChooser<Command> autoChooser;


  //region Trajectories
  // AutoFactory autoFactory = new AutoFactory(
  //   m_drive::getPose, // A function that returns the current robot pose
  //   m_drive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
  //   m_drive::followTrajectory, // The drive subsystem trajectory follower 
  //   true, // If alliance flipping should be enabled 
  //   m_drive);
  
  // Optional<Trajectory<SwerveSample>> testT1 = Choreo.loadTrajectory("Test1");
  // Optional<Trajectory<SwerveSample>> testT2 = Choreo.loadTrajectory("Test2");

  // Command myTrajectory = autoFactory.trajectoryCmd("Test1");
  // Command resetOdometry = autoFactory.resetOdometry("Test1");
  //endregion

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
      m_controller.axisGreaterThan(0, 0.07).or(m_controller.axisGreaterThan(1, 0.07)).or(m_controller.axisGreaterThan(4, 0.07))
      .or(m_controller.axisLessThan(0, -0.07)).or(m_controller.axisLessThan(1, -0.07)).or(m_controller.axisLessThan(4, -0.07))
      .or(AutoAlignLeft).or(AutoAlignRight);
    //  .onTrue(new DriveCommand(m_drive, m_controller, m_elevator, m_limelight));
      resetNavXButton.onTrue(new InstantCommand(m_drive::zeroGyroscope));
    //endregion

    pivotIntakeButton.or(stowedButton).or(L1Button).or(L2Button).or(L3Button).or(AlgaeLowButton).or(AlgaeHighButton).or(ProcessorButton).or(climbSetpointButton).whileFalse(new SnapBack(m_pivot, m_elevator));

    m_controller2.axisGreaterThan(Constants.INTAKE_TRIGGER, 0.7).onTrue( new InstantCommand(m_shintake::intake)).onFalse(new InstantCommand(m_shintake::off));
    m_controller2.axisGreaterThan(Constants.OUTTAKE_TRIGGER, 0.7).onTrue(new InstantCommand(m_shintake::shoot)).onFalse(new InstantCommand(m_shintake::off));
    l1EjectButton.onTrue(new InstantCommand(m_shintake::shootL1)).onFalse(new InstantCommand(m_shintake::off));
    //endregion




    //region Basic Testing Methods
    m_controller2.axisGreaterThan(Constants.PIVOT_JOYSTICK, Constants.PIVOT_DEADBAND).or(m_controller2.axisLessThan(Constants.PIVOT_JOYSTICK, -Constants.PIVOT_DEADBAND)).onTrue(new PivotAnalog(m_pivot, m_controller2)).onFalse(new InstantCommand(m_pivot::off));
    m_controller2.axisGreaterThan(Constants.ELEVATOR_JOYSTICK, Constants.ELEVATOR_DEADBAND).or(m_controller2.axisLessThan(Constants.ELEVATOR_JOYSTICK, -Constants.ELEVATOR_DEADBAND)).onTrue(new ElevateAnalog(m_elevator, m_controller2)).onFalse(new InstantCommand(m_elevator::off));



    //Set Pivot and Elevator Position to Zero
    // climbButton.onTrue(new InstantCommand(m_climber::climb)).onFalse(new InstantCommand(m_climber::off));
    // climbButton.onTrue(new Climb(m_climber, m_pivot, m_controller2)).onFalse(new InstantCommand(m_climber::off));
    // climbUnwind.onTrue(new InstantCommand(m_climber::unwind)).onFalse(new InstantCommand(m_climber::off));
    
    // m_controller2.axisGreaterThan(Constants.CLIMBER_JOYSTICK, Constants.CLIMBER_DEADBAND).or(m_controller2.axisLessThan(Constants.CLIMBER_JOYSTICK, -Constants.CLIMBER_DEADBAND)).onTrue(new Climb(m_climber, m_controller2)).onFalse(new InstantCommand(m_climber::off));
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
    
    Command auto = autoChooser.getSelected();}
    return auto;
}
