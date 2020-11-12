/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The ball hopper subsystem: Used to move balls from the intake near the ground
 * up to the shooter.
 * 
 * The big question for the hopper is where are the balls? The hopper itself can
 * only physically hold 4 balls. The fifth ball will be held in the intake for
 * the total of 5 balls. The ball in the intake is sensed by the intake
 * mIntakeIR digital input (boolean)
 * 
 */
public class Hopper implements Subsystem {
  public int mBallCount = 0;

  private static Hopper instance;
  private DigitalInput mIntakeIR;
  private DigitalInput mOverflowIR;
  private DigitalInput mShooterIR;
  private TalonSRX mMotor1;
  private VictorSPX mMotor2;

  private double motorOutput;

  private Hopper() {
    mMotor1 = new TalonSRX(Constants.Ports.HOPPER_MOTOR_TOP);
    mMotor2 = new VictorSPX(Constants.Ports.HOPPER_MOTOR_BOTTOM);
    mIntakeIR = new DigitalInput(Constants.Ports.INTAKE_IR);
    mShooterIR = new DigitalInput(Constants.Ports.SHOOTER_IR);
    mOverflowIR = new DigitalInput(Constants.Ports.OVERFLOW_IR);

    /* set the motor controller defaults */
    mMotor1.configFactoryDefault(10);
    mMotor2.configFactoryDefault(10);

    mMotor1.setNeutralMode(NeutralMode.Brake);
    mMotor2.setNeutralMode(NeutralMode.Brake);

    mMotor1.setInverted(true);
    mMotor2.setInverted(true);

    mMotor2.follow(mMotor1);

    /* Config the sensor used for Primary PID and sensor direction */
    mMotor1.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 0, 10);

		/* Ensure sensor is positive when output is positive */
		mMotor1.setSensorPhase(true);

		/* Config the peak and nominal outputs, 12V means full */
		mMotor1.configNominalOutputForward(0, Constants.Values.TIMEOUT_MS);
		mMotor1.configNominalOutputReverse(0, Constants.Values.TIMEOUT_MS);
		mMotor1.configPeakOutputForward(1, Constants.Values.TIMEOUT_MS);
		mMotor1.configPeakOutputReverse(-1, Constants.Values.TIMEOUT_MS);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		mMotor1.configAllowableClosedloopError(0, 0, Constants.Values.TIMEOUT_MS);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		mMotor1.config_kF(0, Constants.Values.HOPPER_GAINS.kF, Constants.Values.TIMEOUT_MS);
		mMotor1.config_kP(0, Constants.Values.HOPPER_GAINS.kP, Constants.Values.TIMEOUT_MS);
		mMotor1.config_kI(0, Constants.Values.HOPPER_GAINS.kI, Constants.Values.TIMEOUT_MS);
		mMotor1.config_kD(0, Constants.Values.HOPPER_GAINS.kD, Constants.Values.TIMEOUT_MS);

		/* Reset the quadrature (relative) sensor */
		mMotor1.setSelectedSensorPosition(0, 0, Constants.Values.TIMEOUT_MS);

    SmartDashboard.putNumber("Driver/Set Ball Auto Count", mBallCount);
  }

  /** 
   * Move the hopper belts which will move balls up/down
   * @param speed of the hopper belts
   */
  public void setSpeed(double speed) {
    mMotor1.set(ControlMode.PercentOutput, speed);
  }

  /** 
   * Reset the Talon's internal quad encoder counter.  Note this assumes we are
   * using an external standard quad encoder (like the CUI encoder) and not the CTRE mag encoder.
   */
  public void resetEncoder() {
    mMotor1.setSelectedSensorPosition(0, 0, Constants.Values.TIMEOUT_MS);
  }

  /** 
   * Check if we can sense a ball at the top of the hopper, ready to load into the shooter.
   * If we move this ball any more, it will be pulled out and shot!
   * 
   * @return boolean
   */
  public boolean getShooterBallState() {
    return !mShooterIR.get();
  }

  /** 
   * Check if we can see a ball in the intake (not actually in the hopper).  The ball here will next move into the
   * hopper.
   * @return boolean
   */
  public boolean getIntakeBallState() {
    return !mIntakeIR.get();
  }
  
  /** 
   * Now I don't really know where this sensor is at.  I would expect it to be located at the bottom of the hopper.
   * @return boolean
   */
  public boolean getOverflowBallState() {
    return !mOverflowIR.get();
  }

  /**
   * Send some of the hopper variables to the smartdashboard.
   */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Hopper/Ball Count", mBallCount);
    SmartDashboard.putBoolean("Hopper/Intake IR Sensor", getIntakeBallState());
    SmartDashboard.putBoolean("Hopper/Shooter IR Sensor", getShooterBallState());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();

 		/* Get Talon/Victor's current output percentage */
    motorOutput = mMotor1.getMotorOutputPercent();

  }

  /** 
   * @return Hopper
   */
  public static Hopper getInstance() {
    if (instance == null)
      instance = new Hopper();
    return instance;
  }
}
