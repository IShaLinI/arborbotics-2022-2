package frc.robot.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class RedSingleSteal extends SequentialCommandGroup {
    public RedSingleSteal(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        IntakePistonsSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator,
        HoodSubsystem hood,
        VisionSupplier vision
    ) {
        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(PATH_TO_RED_2.getInitialPose()), drivetrain), //Reset Position
            new IntakePath(PATH_TO_RED_2, drivetrain, intake, pistons), //Intake Red 2
            new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
            new FireRoutine(flywheel, hood, accelerator, 0.5), //Fire Red 1 & 2
            new IntakePath(PATH_TO_BLUE_1, drivetrain, intake, pistons), //Intake Blue 1
            new ParallelCommandGroup( //Arrange Shooter
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_BLUE_SHOT_1), //Move to firing position
                new InstantCommand(() -> hood.setTargetAngle(45), hood),
                new InstantCommand(() -> flywheel.setTargetRPM(1000), flywheel)
            ),
            new FireRoutine(flywheel, hood, accelerator, 0.5) //Fire Blue 1
        );
    }

    private Trajectory PATH_TO_RED_2 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(10.5, 3.55, Rotation2d.fromDegrees(-89.15)),
        new Pose2d(11.58, 2.125, Rotation2d.fromDegrees(-30))

    ),
    false, 
    "Red Single Steal PATH_TO_RED_2"
    );

    private Trajectory PATH_TO_BLUE_1 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(10.5, 3.55, Rotation2d.fromDegrees(-89.15)),
        new Pose2d(10.5, 1.7, Rotation2d.fromDegrees(-90))

    ),
    false, 
    "Red Hangar 2 PATH_TO_BLUE_1"
    );

    private Trajectory PATH_TO_BLUE_SHOT_1 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(10.5, 1.7, Rotation2d.fromDegrees(180)),
        new Pose2d(11.5, 1.7, Rotation2d.fromDegrees(180))
    ),
    true, 
    "Red Hangar 2 PATH_TO_BLUE_SHOT_1"
    );

}

