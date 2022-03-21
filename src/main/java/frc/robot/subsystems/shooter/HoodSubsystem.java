package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.custom.ArborMath;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class HoodSubsystem extends SubsystemBase implements Loggable{

  private final CANSparkMax mMotor = new CANSparkMax(Constants.CAN.kHood, MotorType.kBrushless);

  private PIDController mPID = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward mFeedForward = Constants.Flywheel.kFeedForward;

  public double mTargetAngle, mCurrentAngle, mPIDEffort, mFFEffort;

  public HoodSubsystem() {
    configureMotor();
  }

  public void configureMotor(){
    mMotor.restoreFactoryDefaults();
    mMotor.setIdleMode(IdleMode.kBrake);
    mMotor.getEncoder().setPositionConversionFactor(Constants.Hood.kAngleConversionFactor);
  }
  
  public void runHood(){

    mCurrentAngle = getAngle();

    if(mTargetAngle != 0){
      mFFEffort = mFeedForward.calculate(mTargetAngle);
      mPIDEffort = mPID.calculate(mCurrentAngle, mTargetAngle);
    }else{
      mFFEffort = 0;
      mPIDEffort = 0;
    }

    mMotor.setVoltage(mPIDEffort + mFFEffort);

  }

  public void setTargetAngle(double newTarget){
    mTargetAngle = newTarget;
  }

  @Log(tabName = "Shooter")
  public double getAngle(){
    return mMotor.getEncoder().getPosition();
  }

  public boolean ready(){
    return ArborMath.inTolerance(Math.abs(mTargetAngle-mCurrentAngle), 50);
  }

  public void stop(){
    setTargetAngle(0);
    mMotor.stopMotor();
  }
}
