// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.Interpolation.InterpolatingTable;


/**
 * <p>
 * Aim and set hood angle based on vision measurements.
 * </p>
 * 
 * Note: This will instantly cancel if no target is found
 * 
 */
public class AimRoutine extends ParallelRaceGroup {
  public AimRoutine(DrivetrainSubsystem drivetrain, HoodSubsystem hood, FlywheelSubsystem flywheel, VisionSupplier vision) {
    addCommands(
      new WaitUntilCommand(() -> Math.abs(vision.getYaw()) < 0.1),
      new RunCommand(() -> flywheel.setTargetRPM(InterpolatingTable.get(vision.getDistance()).rpm), flywheel),
      new RunCommand(() -> hood.setTargetAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle), hood)
      //drivetrain.new VisionAimAssist()
    );
  }
}
