// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

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

  // JoystickButton shootHighButton = new JoystickButton(m_controller2.getHID(), 6);
  // JoystickButton shootLowButton = new JoystickButton(m_controller2.getHID(), 5);
  JoystickButton intakeButton = new JoystickButton(m_controller2.getHID(), 2);
  JoystickButton groundIntakeButton = new JoystickButton(m_controller2.getHID(), 6);
  JoystickButton reverseIntakeButton = new JoystickButton(m_controller2.getHID(), 1);
  JoystickButton groundIntakeUpButton = new JoystickButton(m_controller2.getHID(), 5);
  // JoystickButton shootButton = new JoystickButton(m_controller2.getHID(), 8);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  //region Subsystems
  private final AHRS m_ahrs = new AHRS(NavXComType.kMXP_SPI);
  public final DrivetrainSubsystem m_drive = new DrivetrainSubsystem(m_ahrs);
  private final Limelight m_limelight = new Limelight(m_drive, m_controller);
  private final Intake m_intake = new Intake();
  private final Shooter m_Shooter = new Shooter();
  private final Belt m_Belt = new Belt();
  private final GroundIntake m_pivot = new GroundIntake();
  private final Candle m_leds = new Candle();
  //endregion 

  //AUTOCHOOSER SET UP
  private final SendableChooser<Command> autoChooser;

  


  //region Trajectories
  

  public RobotContainer() {
    // Configure the trigger bindings
    
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("RunBelt", new InstantCommand(m_Belt::runBelt));
    NamedCommands.registerCommand("StopBelt", new InstantCommand(m_Belt::off));
    NamedCommands.registerCommand("ShootHigh", new InstantCommand(m_Shooter::shootHigh));
    NamedCommands.registerCommand("Intake", new InstantCommand(m_intake::intake));

      // 1. COMPLEX SHOT COMMAND
    // Logic: Belt runs for 3.7s total. At 0.7s mark, Shooter/Intake join in for 3.0s.
    NamedCommands.registerCommand("ComplexShot", 
        Commands.parallel(
            // PROCESS A: The Belt (Runs for the whole duration: 0.7 delay + 3.0 shoot)
            // We use startEnd to ensure it turns OFF automatically when the timeout finishes
            // Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt)
            //     .withTimeout(4.0),
            Commands.startEnd(m_Shooter::shootHigh, m_Shooter::off, m_Shooter)
                .withTimeout(4),

            // PROCESS B: The Delays and other systems
            Commands.sequence(
                new WaitCommand(1.0), // Wait 0.7 seconds while belt is running alone
                
                // Now start the Shooter and Intake together
                Commands.parallel(
                    Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt),
                    Commands.startEnd(m_intake::intake, m_intake::off, m_intake)
                ).withTimeout(3.0) // They run for 3 seconds then stop
            )
        )
    );

    // 2. GROUND INTAKE CYCLE
    // Logic: Pivot down, run intake, pivot up.
    // NamedCommands.registerCommand("GroundIntakeCycle", 
    //     Commands.sequence(
    //         // Step 1: Pivot Down (Assuming this is instant or needs a small wait to settle)
    //         Commands.runOnce(m_pivot::pivotDown, m_pivot),
    //         new WaitCommand(0.5), // Give it 0.5s to actually physically lower
            
    //         // Step 2: Run the intake rollers for 2.0 seconds (or however long you need)
    //         Commands.startEnd(m_pivot::IntakeOn, m_pivot::off, m_pivot)
    //             .withTimeout(2.0),

    //         // Step 3: Pivot back up
    //         Commands.runOnce(m_pivot::pivotUp, m_pivot)
    //     )
    // );

    NamedCommands.registerCommand("GroundIntakeCycle", 
    Commands.sequence(
        // 1. Pivot Down
        Commands.run(m_pivot::pivotDownAuto, m_pivot)
            .until(() -> m_pivot.getPivotPosition() >= Constants.PIVOT_DOWN_POSITION),
        
        // 2. Run Intake UNTIL note is found OR time runs out
        Commands.run(m_pivot::IntakeOn, m_pivot)
            .until(m_pivot::hasNote),  // <--- THE KEY FIX: Stops the moment sensor triggers
            // .withTimeout(3.0),        // <--- SAFETY NET: Stops if we miss the ball after 3s

        // 3. Force Stop Intake (Crucial to ensure it doesn't keep coasting)
        Commands.runOnce(m_pivot::IntakeOff, m_pivot),

        // 4. Pivot Up
        Commands.run(m_pivot::pivotUpAuto, m_pivot)
             .until(() -> m_pivot.getPivotPosition() <= Constants.PIVOT_UP_POSITION + 0.5),

        Commands.run(m_pivot::IntakeReverse, m_pivot)
          .withTimeout(0.5)
    )
);
    // --- REGISTER NAMED COMMANDS FOR PATHPLANNER ---
    // m_leds.setIdleRainbow();
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
      .onTrue(new DriveCommand(m_drive, m_controller, m_limelight));
    // m_drive.setDefaultCommand(new DriveCommand(m_drive, m_controller, m_limelight));
    resetNavXButton.onTrue(new InstantCommand(m_drive::zeroGyroscope));
    // m_controller.a().whileTrue(new TestMotorCommand(m_drive));

    //endregion

    
    
    //region Shooter, Intake, and Belt
    //shootHighButton.onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootHigh), new InstantCommand(m_Belt::runBelt), new InstantCommand(m_intake::intake))).onFalse(Commands.parallel(new InstantCommand(m_Shooter::off), new InstantCommand(m_Belt::off), new InstantCommand(m_intake::off)));
    //shootLowButton.onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootLow), new InstantCommand(m_Belt::runBelt), new InstantCommand(m_intake::intake))).onFalse(Commands.parallel(new InstantCommand(m_Shooter::off), new InstantCommand(m_Belt::off), new InstantCommand(m_intake::off)));
    m_controller2.axisGreaterThan(2 , 0.7)
    .onTrue(Commands.parallel(
    new InstantCommand(m_Shooter::shootHigh),
    new InstantCommand(m_leds::flameOn),
    // new InstantCommand(m_leds:: setGreen),
    // new InstantCommand(m_intake::intake),
    new RunCommand(() -> {}, m_Belt).withTimeout(0.4)
        .andThen(new InstantCommand(m_Belt::runBelt))
))  

    .onFalse(Commands.parallel(new InstantCommand(m_Belt::off), new InstantCommand(m_Shooter::off), new InstantCommand(m_intake::off)));
    m_controller2.axisGreaterThan(3, 0.7).onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootLow), new InstantCommand(m_Belt::runBelt))).onFalse(Commands.parallel(new InstantCommand(m_Shooter::off), new InstantCommand(m_Belt::off), new InstantCommand(m_intake::off)));
    intakeButton.whileTrue(new RunCommand(m_intake::intake, m_intake)).onFalse(new InstantCommand(m_intake::off));
    reverseIntakeButton.whileTrue(new RunCommand(m_intake::reverse, m_intake)).onFalse(new InstantCommand(m_intake::off));
    //groundIntakeButton.onTrue(new InstantCommand(m_))
    //shootHighButton.onTrue(Commands.parallel(new InstantCommand(m_Shooter::shootHigh), new InstantCommand(m_Belt::runBelt), new InstantCommand(m_intake::intake))).onFalse(Commands.parallel(new InstantCommand(m_Shooter::off), new InstantCommand(m_Belt::off), new InstantCommand(m_intake::off)));
    //endregion

   groundIntakeButton.whileTrue(new RunCommand(m_pivot::pivotDown, m_pivot)).onFalse(new InstantCommand(m_pivot::off)); 
    //new RunCommand(m_pivot::IntakeOn, m_pivot)
    // new RunCommand(m_leds:: setIntaking, m_leds)
// "Up" Intake Button
   groundIntakeUpButton.whileTrue(new RunCommand(m_pivot::pivotUp, m_pivot)).onFalse( new InstantCommand(m_pivot::off)); 



    //region Basic Testing Methods
 
    // groundIntakeButton.onTrue(Commands.sequence(new InstantCommand(m_pivot::pivotDown), new InstantCommand(m_pivot::IntakeOn))).onFalse(new InstantCommand(m_pivot::off));
    // groundIntakeUpButton.onTrue(Commands.sequence(new InstantCommand(m_pivot::pivotUp), new InstantCommand(m_pivot:: IntakeOn))).onFalse(new InstantCommand(m_pivot::off));
    
    // shootButton.whileTrue(new InstantCommand(m_Belt::runBelt));
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
    System.out.println("getAutonomousCommand");
    return new PathPlannerAuto("Tester");
    
  }
};
