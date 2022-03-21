package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.custom.ArborMath;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class FlywheelSubsystem extends SubsystemBase implements Loggable {

  private final WPI_TalonFX mMotor = new WPI_TalonFX(Constants.CAN.kFlywheelMaster);
  private final TalonFXSimCollection mMotorSim = mMotor.getSimCollection();

  private PIDController mPID = new PIDController(0.2, 0, 0);
  private final SimpleMotorFeedforward mFeedForward = Constants.Flywheel.kFeedForward;

  public double mTargetRPM, mCurrentRPM, mPIDEffort, mFFEffort;

  public static final LinearSystem<N1, N1, N1> kPlant = LinearSystemId.identifyVelocitySystem(12d/6380d, 0.009318);

  private FlywheelSim mSim = new FlywheelSim(
    kPlant,
    DCMotor.getFalcon500(1),
    1/1.5
  );

  public FlywheelSubsystem() {
    configureMotor();
    mTargetRPM = 0;
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

    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

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

  @Log(tabName = "Shooter")
  public double getRPM(){
    return ((mMotor.getSelectedSensorVelocity() * 60) / 2048) * 1.5;
  }

  public boolean ready(){
    return ArborMath.inTolerance(Math.abs(mTargetRPM-mCurrentRPM), 50) && mTargetRPM != 0;
  }

  public void stop(){
    setTargetRPM(0);
    mMotor.stopMotor();
  }

  @Override
  public void periodic() {
      if(mTargetRPM != 0){
        runFlywheel();
      }else{
        stop();
      }
  }

  @Override
  public void simulationPeriodic() {

    mSim.setInput(mMotor.get() * RobotController.getInputVoltage());

    mSim.update(0.02);

    double flywheelNativeVelocity = mSim.getAngularVelocityRPM() * 2048 / (60 * 10) * 1d/1.5;
    double flywheelNativePositionDelta = flywheelNativeVelocity*10*0.02;

    mMotorSim.setIntegratedSensorVelocity((int)flywheelNativeVelocity);
    mMotorSim.addIntegratedSensorPosition((int)flywheelNativePositionDelta);

    mMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

  }

  @Log(tabName = "Shooter")
  private double getRPMTarget(){
    return mTargetRPM;
  }

}
