package frc.robot.autonomous.routines.BlueAllience;

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

public class BlueMidTwo extends SequentialCommandGroup{
    public BlueMidTwo(
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
          new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Blue 2
          new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
          new FireRoutine(flywheel, hood, accelerator, 0.5) //Fire Blue 1 & 2
        );
    }

    private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1,1,List.of(
        new Pose2d(6.666, 2.737, Rotation2d.fromDegrees(200)),
        new Pose2d(5.164,2.044,Rotation2d.fromDegrees(200))
    ),
    false,
    "Blue Mid Two PATH_TO_BALL_2"
    );
}
