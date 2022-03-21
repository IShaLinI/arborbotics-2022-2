// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.Interpolation.InterpolatingTable;

public class FireRoutine extends SequentialCommandGroup {
  public FireRoutine(FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator, VisionSupplier vision) {
    addCommands(

      new InstantCommand(() -> flywheel.setTargetRPM(InterpolatingTable.get(vision.getDistance()).rpm), flywheel),
      new InstantCommand(() -> hood.setTargetAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle), flywheel),

      new ConditionalCommand(
        new ParallelCommandGroup(
          new InstantCommand(accelerator::start, accelerator),
          new RunCommand(flywheel::runFlywheel, flywheel),
          new RunCommand(hood::runHood, hood)
        ),
        new ParallelCommandGroup(
          new RunCommand(() -> flywheel.setTargetRPM(InterpolatingTable.get(vision.getDistance()).rpm), flywheel),
          new RunCommand(hood::runHood, hood)
        ),
        () -> (flywheel.ready() && hood.ready())
      ).andThen(
        new ParallelCommandGroup(
          new InstantCommand(flywheel::stop, flywheel),
          new InstantCommand(hood::stop, hood),
          new InstantCommand(accelerator::stop, accelerator)
        )
      )
    );
  }
}
