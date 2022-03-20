package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.custom.ArborMath;

public class FlywheelSubsystem extends SubsystemBase {

  private final WPI_TalonFX mMotor = new WPI_TalonFX(Constants.CAN.kFlywheelMaster);

  private PIDController mPID = new PIDController(Constants.Flywheel.kP, 0, 0);
  private final SimpleMotorFeedforward mFeedForward = Constants.Flywheel.kFeedForward;

  public double mTargetRPM, mCurrentRPM, mPIDEffort, mFFEffort;

  public FlywheelSubsystem() {
    configureMotor();

    if(Constants.Telemetry.Flywheel){
      Shuffleboard.getTab("Shooter").add("Flywheel RPM", getRPM());
      Shuffleboard.getTab("Shooter").add("Flywheel Spun-up", ArborMath.inTolerance(getRPM(), Constants.Flywheel.kRPMTolerance));
    }

  }

  public void configureMotor(){
    mMotor.configFactoryDefault();
    mMotor.setNeutralMode(NeutralMode.Coast);

    mMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    mMotor.configVelocityMeasurementWindow(1);

    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 250);

  }
  
  public void runFlywheel(){

    mCurrentRPM = getRPM();

    if(mTargetRPM != 0){
      mFFEffort = mFeedForward.calculate(mTargetRPM);
      mPIDEffort = mPID.calculate(mCurrentRPM, mTargetRPM);
    }else{
      mFFEffort = 0;
      mPIDEffort = 0;
    }

    mMotor.setVoltage(mPIDEffort + mFFEffort);

  }

  public void setTargetRPM(double newTarget){
    mTargetRPM = newTarget;
  }

  public double getRPM(){
    return mMotor.getSelectedSensorVelocity() * Constants.Flywheel.kRPMConversionFactor;
  }

  public boolean ready(){
    return ArborMath.inTolerance(Math.abs(mTargetRPM-mCurrentRPM), 50);
  }

  public void stop(){
    setTargetRPM(0);
    mMotor.stopMotor();
  }

}
