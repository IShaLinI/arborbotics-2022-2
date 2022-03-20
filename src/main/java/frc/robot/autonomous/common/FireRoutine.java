// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class FireRoutine extends SequentialCommandGroup {
  public FireRoutine(FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator, double shotDuration) {
    addCommands(
      new ConditionalCommand(
        new StartEndCommand(
          accelerator::start,
          accelerator::stop,
          accelerator
        ).withTimeout(shotDuration),
        new InstantCommand(),
        () -> (flywheel.ready() && hood.ready())
      ).andThen(
        new ParallelCommandGroup(
          //hood.new Home(),
          new InstantCommand(flywheel::stop, flywheel)
        )
      )      
    );
  }
}
