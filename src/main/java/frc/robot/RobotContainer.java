package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

  private SendableChooser<SequentialCommandGroup> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    configureAutoChooser();
  }

  private void configureButtonBindings() {}
  
  public void configureAutoChooser(){
    mAutoChooser.setDefaultOption("Nothing", null);
  }


  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
  }
  public void stopAll(){
    new ParallelCommandGroup(

    ).schedule();
  }
}
