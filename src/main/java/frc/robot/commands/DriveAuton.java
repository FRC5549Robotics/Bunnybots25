package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAuton extends Command {
  private ChassisSpeeds chassisSpeeds;

  // The subsystem the command runs on
  public final DrivetrainSubsystem drivetrain;
  Timer timer;
  Optional<Trajectory<SwerveSample>> traj;
  double startTime;
  

  public DriveAuton(DrivetrainSubsystem subsystem, Optional<Trajectory<SwerveSample>> Traj){
      drivetrain = subsystem;
      timer = new Timer();
      traj = Traj;
      addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.reset();
    startTime = timer.get();
    Optional<Pose2d> initialPose = traj.get().getInitialPose(DriverStation.getAlliance().get() == Alliance.Red);
    if (initialPose.isPresent()) {
      drivetrain.resetOdometry(initialPose.get());
    }
  }

          
  @Override
  public void execute(){
      drivetrain.followTrajectory(traj.get().sampleAt(timer.get() - startTime, DriverStation.getAlliance().get() == Alliance.Red).get());
      Optional<SwerveSample> sample = traj.get().sampleAt(timer.get(),  DriverStation.getAlliance().get() == Alliance.Red);
    }

  @Override
  public boolean isFinished() {
    double x = drivetrain.getPose().getMeasureX().abs(Meter) - traj.get().getFinalPose(DriverStation.getAlliance().get() == Alliance.Red).get().getMeasureX().abs(Meter);
    double y = drivetrain.getPose().getMeasureY().abs(Meter) - traj.get().getFinalPose(DriverStation.getAlliance().get() == Alliance.Red).get().getMeasureY().abs(Meter);
    double theta = Math.cos(drivetrain.getPose().getRotation().getRadians() - traj.get().getFinalPose(DriverStation.getAlliance().get() == Alliance.Red).get().getRotation().getRadians());
    return Math.abs(x) < 0.02 && 
            Math.abs(y) < 0.02 && 
            Math.abs(theta - 1) < 0.0038053;
  }

  
  @Override
  public void end(boolean interrupted) {
    drivetrain.snap();
  }
}