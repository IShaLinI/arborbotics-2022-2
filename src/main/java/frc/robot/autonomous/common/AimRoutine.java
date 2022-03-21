// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;

public class AimRoutine extends ParallelRaceGroup {
  public AimRoutine(DrivetrainSubsystem drivetrain, VisionSupplier vision) {
    addCommands(
      new WaitUntilCommand(() -> Math.abs(vision.getYaw()) < 0.5),
      drivetrain.new VisionAimAssist(vision)
    );
  }
}
