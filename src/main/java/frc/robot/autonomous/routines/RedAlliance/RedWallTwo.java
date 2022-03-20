// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.AimRoutine;
import frc.robot.autonomous.common.FireRoutine;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class RedWallTwo extends SequentialCommandGroup {
  public RedWallTwo(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    IntakePistonsSubsystem pistons,
    FlywheelSubsystem flywheel,
    AcceleratorSubsystem accelerator,
    HoodSubsystem hood,
    VisionSupplier vision
  ) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(PATH_TO_BALL_2.getInitialPose()), drivetrain), //Reset Position
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Red 2
      new AimRoutine(drivetrain, hood, flywheel, vision).withTimeout(1.5), //Aim
      new FireRoutine(flywheel, hood, accelerator, 1.5).withTimeout(1.5), //Fire Red 2
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_TAXI) //Ensure Taxi
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(8.85, 6.36, Rotation2d.fromDegrees(90.7)),
    new Pose2d(8.93, 7.34, Rotation2d.fromDegrees(90))
  ), false, 
  "Red Wall Two PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_TAXI = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(8.93, 7.34, Rotation2d.fromDegrees(180)),
    new Pose2d(11,7.5, Rotation2d.fromDegrees(180))
  ), true, 
  "Red Wall Two PATH_TO_TAXI"
  );

}
