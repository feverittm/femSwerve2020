/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Gains;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax mMotor1;
  private CANSparkMax mMotor2;
  private CANPIDController mController;
  private CANEncoder mEncoder;
  private double setpoint;

  /**
   * Create the ShooterSubsystem. Design Notes: 4in Fairlane wheel driven by 2
   * parallel NEO motors through a 18:22 gearing. Output from other side of
   * Farlane wheel connects to a flywheel with I=~ 3lbs/sec.in through a 2:1
   * upspeed gearing. NEO driven by SparkMax via CAN, internal NEO encoder.
   */
  public ShooterSubsystem() {
    mMotor1 = new CANSparkMax(Constants.Ports.SHOOTER_MOTOR_1, MotorType.kBrushless);
    mMotor2 = new CANSparkMax(Constants.Ports.SHOOTER_MOTOR_2, MotorType.kBrushless);

    mMotor1.restoreFactoryDefaults();
    mMotor2.restoreFactoryDefaults();

    mMotor1.setIdleMode(IdleMode.kCoast);
    mMotor2.setIdleMode(IdleMode.kCoast);

    mMotor2.follow(mMotor1);

    mEncoder = mMotor1.getEncoder();
    mEncoder.setVelocityConversionFactor(Constants.Values.SHOOTER_GEARING);

    mController = mMotor1.getPIDController();
    mController.setP(Constants.Values.SHOOTER_VELOCITY_GAINS.kP);
    mController.setI(Constants.Values.SHOOTER_VELOCITY_GAINS.kI);
    mController.setD(Constants.Values.SHOOTER_VELOCITY_GAINS.kD);
    mController.setFF(Constants.Values.SHOOTER_VELOCITY_GAINS.kF);
    mController.setIZone(0.0);
    mController.setOutputRange(0, 1); // Don't spin shooter backwards

  }

  /*
   * Set the speed of the shooter wheel (hold via velocity PID)
   * 
   * @param rpm Set the speed (in rpm) of the shiiter wheel
   */
  public void setRPM(double rpm) {
    mController.setReference(rpm, ControlType.kVelocity);
    setpoint = rpm;
  }

  /*
   * Force the speed of the shooter wheel directly via percent power
   * 
   * @param perc Set the percentage of power for the shooter wheel
   */
  public void setPercent(double perc) {
    mController.setReference(perc, ControlType.kDutyCycle);
    setpoint = 0.0;
  }

  /*
   * Stop the shooter wheel
   */
  public void goodStop() {
    setpoint = 0.0;
    mMotor1.set(0.0);
    mController.setReference(0.0, ControlType.kDutyCycle);
  }

  /**
   * Get the speed/rpm of the shooter wheel
   * 
   * @return rpm
   */
  public double getRPMs() {
    return mEncoder.getVelocity();
  }

  /**
   * Given a distance, calculate the required shooter rpm
   * 
   * @param distance
   * @return rpm
   */
  public double getNeededBallVelocity(double distance) {
    return (1 / ((Math.tan(Constants.Values.SHOOTER_RELEASE_ANGLE * (Math.PI / 180)) / (-4.9 * distance))
        + ((2.49 - Constants.Values.SHOOTER_RELEASE_HEIGHT) / (4.9 * distance * distance))))
        * (1 / Math.cos(Constants.Values.SHOOTER_RELEASE_ANGLE * (Math.PI / 180)));
  }

  /**
   * @return the linear speed of a ball
   */
  public double getBallSpeed() {
    return (getRPMs() / 60) * (Constants.Values.SHOOTER_CIRCUMFERENCE_CM / 100);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Shooter/encoderspeed", getRPMs());
    SmartDashboard.putNumber("Shooter/Ball Ejection Speed", getBallSpeed());
    
    // for tuning the PID controller
    SmartDashboard.putNumber("SetPoint", setpoint);
    SmartDashboard.putNumber("ProcessVariable", mEncoder.getVelocity());
    SmartDashboard.putNumber("Controller Output", mMotor1.getAppliedOutput());
  }

  public Gains getGains() {
    return new Gains(mController.getP(), mController.getI(), mController.getD(), mController.getFF());
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getOutput() {
    return mMotor1.getAppliedOutput();
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  private static ShooterSubsystem instance;

  /**
   * @return ShooterSubsystem
   */
  public static ShooterSubsystem getInstance() {
    if (instance == null)
      instance = new ShooterSubsystem();
    return instance;
  }
}
