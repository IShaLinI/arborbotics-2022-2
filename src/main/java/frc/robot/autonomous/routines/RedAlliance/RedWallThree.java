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

public class RedWallThree extends SequentialCommandGroup {
  public RedWallThree(
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
      new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
      new FireRoutine(flywheel, hood, accelerator, 0.5), //Fire Red 1 & 2
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons), //intake Red 3
      new AimRoutine(drivetrain, hood, flywheel, vision), //Aim 
      new FireRoutine(flywheel, hood, accelerator, 0.5) //Fire Red 3
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(2,3,List.of(
    new Pose2d(8.91, 6, Rotation2d.fromDegrees(91.158)),
    new Pose2d(8.962, 7.4, Rotation2d.fromDegrees(91.158))
  ),
  false,
  "Red Wall Three TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(9, 7.3, Rotation2d.fromDegrees(-90)),
    new Pose2d(11.5, 6.4, Rotation2d.fromDegrees(35))
  ),
  false,
  "Red Wall Three TWO PATH_TO_BALL_3"
  );

}
