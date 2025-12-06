// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class TestMotorCommand extends Command {
//     private final DrivetrainSubsystem m_drive;

//     public TestMotorCommand(DrivetrainSubsystem drive) {
//         m_drive = drive;
//         addRequirements(m_drive);
//     }

//     @Override
//     public void initialize() {
//         System.out.println("--- STARTING MOTOR TEST ---");
//     }

//     @Override
//     public void execute() {
//         // Test: 20% forward power, steer to 90 degrees
//         // m_drive.rawDriveTestFrontLeft(0.2);
//         m_drive.testFrontLeftMotor(0.2, 90);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Stop the motor when the command ends
//         m_drive.testFrontLeftMotor(0,0);
//         System.out.println("--- ENDING MOTOR TEST ---");
//     }
// }