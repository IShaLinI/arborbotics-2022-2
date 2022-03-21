package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.routines.test.RoutineTesting;
import frc.robot.custom.ArborMath;
import frc.robot.custom.controls.CommandXboxController;
import frc.robot.custom.controls.Deadbander;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class RobotContainer {

  private VisionSubsystem vision = new VisionSubsystem();
  private AcceleratorSubsystem accelerator = new AcceleratorSubsystem();
  private LiftSubsystem lift = new LiftSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private IntakePistonsSubsystem pistons = new IntakePistonsSubsystem();
  private FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private HoodSubsystem hood = new HoodSubsystem();
  private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  private Trigger intakeExtended = new Trigger(pistons::extended);

  private SendableChooser<SequentialCommandGroup> mAutoChooser = new SendableChooser<>();

  private static Field2d mField = new Field2d();
  private FieldObject2d mIntakeVisualizer = mField.getObject("Intake");
  private SlewRateLimiter mIntakeAnimator = new SlewRateLimiter(1);

  private CommandXboxController mDriverController = new CommandXboxController(0);

  public RobotContainer() {

    LiveWindow.disableAllTelemetry();

    configureButtonBindings();
    configureAutoChooser();

    SmartDashboard.putData(mField);

    drivetrain.setDefaultCommand(
      drivetrain.new TeleOpCommand(
        () -> ArborMath.signumPow(Deadbander.applyLinearScaledDeadband(-mDriverController.getLeftY(), 0.075), 1.2),
        () -> ArborMath.signumPow(Deadbander.applyLinearScaledDeadband(-mDriverController.getRightX(), 0.075), 1.2), 
        () -> mDriverController.getLeftTrigger()
      )
    );

    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.6");

  }

  private void configureButtonBindings() {

    mDriverController.rightBumper().whenHeld(
      new StartEndCommand(
        pistons::extend, 
        pistons::retract,
        pistons
      )
    );

    mDriverController.a().whenHeld(
      drivetrain.new VisionAimAssist(vision.visionSupplier)
    );

    mDriverController.b().whenPressed(
        new ParallelCommandGroup(
          new InstantCommand(() -> flywheel.setTargetRPM(6380*1.5), flywheel),
          new InstantCommand(() -> hood.setTargetAngle(10), hood)
        )
    );

  }
  
  public void configureAutoChooser(){
    mAutoChooser.setDefaultOption("Nothing", null);
    mAutoChooser.addOption("Test", 
      new RoutineTesting(drivetrain, intake, pistons, flywheel, hood, accelerator, vision.visionSupplier)
    );
    SmartDashboard.putData("Auto Chooser", mAutoChooser);
  }

  public void updateField(){
    if(intakeExtended.getAsBoolean()){
      mIntakeVisualizer.setPose(
        new Pose2d(
          (drivetrain.getRobotPosition().getX() + mIntakeAnimator.calculate(0.6056505) * Math.cos(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          (drivetrain.getRobotPosition().getY() + mIntakeAnimator.calculate(0.6056505) * Math.sin(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          drivetrain.getRobotPosition().getRotation()
        )
      );
    }else{
      mIntakeVisualizer.setPose(
        new Pose2d(
          (drivetrain.getRobotPosition().getX() + mIntakeAnimator.calculate(0.3796505) * Math.cos(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          (drivetrain.getRobotPosition().getY() + mIntakeAnimator.calculate(0.3796505) * Math.sin(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          drivetrain.getRobotPosition().getRotation()
        )
      );
    }

    mField.setRobotPose(drivetrain.getRobotPosition());

    vision.visionSupplier.processSim(drivetrain.getRobotPosition());

  }


  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
  }

  public static Field2d getField(){
    return mField;
  }

  public Command stopAll(){
      return new ParallelCommandGroup(
        new InstantCommand(intake::stop, intake),
        new InstantCommand(pistons::retract, pistons),
        new InstantCommand(accelerator::stop, accelerator),
        new InstantCommand(lift::stop, lift),
        new InstantCommand(flywheel::stop, flywheel),
        new InstantCommand(vision.visionSupplier::disableLEDs, vision),
        new InstantCommand(drivetrain::stop, drivetrain),
        new InstantCommand(hood::stop, hood),
        new PrintCommand("Robot Disabled")
      );
  }
}
