/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;

import org.frcteam2910.common.drivers.SwerveModule;

/**
 * Driver for Team 997's 2020 revision of the 2910 swerve module.  Leveraged from team 3663 code.
 * <p>
 * This implementation assumes that two Spark MAXs are used to control each
 * module over CAN and that the angle Spark MAX is connected to an analog
 * encoder.
 * <p>
 * The drive distance units default to inches. This can be changed using
 * {@link SpartanSwerveModule#setDriveTicksPerUnit(double)}
 */
public final class SpartanSwerveModule extends SwerveModule {

    private static final double ANGLE_TOTAL_GEAR_RATIO = 63.0;

    private static final double DRIVE_GEAR_RATIO = 8.75;

    private static final double DEFAULT_WHEEL_REVOLUTIONS_PER_UNIT = .076;

    private final double offsetAngle;

    private final CANSparkMax angleMotor;
    private final TalonFX driveMotor;

    private final CANAnalog angleAbsoluteEncoder;
    private final CANEncoder angleMotorEncoder;
    private final PIDController anglePIDController;

    private final double ANGLE_P = .5;
    private final double ANGLE_I = 0.0;
    private final double ANGLE_D = 0.0;

    /**
     * The amount of drive encoder ticks that occur for one unit of travel.
     *
     * @see #DEFAULT_WHEEL_REVOLUTIONS_PER_UNIT
     */
    private volatile double wheelRevolutionsPerUnit = DEFAULT_WHEEL_REVOLUTIONS_PER_UNIT;

    /**
     * @param modulePosition the module's offset from the center of the robot
     * @param offsetAngle    how much to offset the angle encoder by in radians
     * @param angleMotor     the motor controller that controls the angle motor
     * @param driveMotor     the motor controller that controls the drive motor
     */
    public SpartanSwerveModule(final Vector2 modulePosition, final double offsetAngle, final CANSparkMax angleMotor,
            final TalonFX driveMotor) {
        super(modulePosition);
        this.offsetAngle = offsetAngle;
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;

        // Configure absolute encoder
        this.angleAbsoluteEncoder = new CANAnalog(this.angleMotor, AnalogMode.kAbsolute);
        this.angleAbsoluteEncoder.setInverted(false);

        // Configure angle motor encoder
        angleMotorEncoder = angleMotor.getEncoder();
        angleMotorEncoder.setPositionConversionFactor(2 * Math.PI / ANGLE_TOTAL_GEAR_RATIO);
        angleMotorEncoder.setPosition(readAngle());

        // Configure PID controllers
        anglePIDController = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D);
        anglePIDController.enableContinuousInput(0, 2 * Math.PI);

        // Config angle motor
        this.angleMotor.setIdleMode(IdleMode.kBrake);

        // Configure drive motor
        driveMotor.configFactoryDefault();
        driveMotor.setInverted(false);
        driveMotor.configNeutralDeadband(0.001);
        
        /* Config sensor used for Primary PID [Velocity] */
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Values.TIMEOUT_MS);
        driveMotor.setSelectedSensorPosition(0);

        /* Config the peak and nominal outputs */
		driveMotor.configNominalOutputForward(0, Constants.Values.TIMEOUT_MS);
	    driveMotor.configNominalOutputReverse(0, Constants.Values.TIMEOUT_MS);
		driveMotor.configPeakOutputForward(1, Constants.Values.TIMEOUT_MS);
		driveMotor.configPeakOutputReverse(-1, Constants.Values.TIMEOUT_MS);

        driveMotor.config_kP(0, Constants.Values.DRIVE_VELOCITY_GAINS.kP, Constants.Values.TIMEOUT_MS);
        driveMotor.config_kI(0, Constants.Values.DRIVE_VELOCITY_GAINS.kI, Constants.Values.TIMEOUT_MS);
        driveMotor.config_kD(0, Constants.Values.DRIVE_VELOCITY_GAINS.kD, Constants.Values.TIMEOUT_MS);
        driveMotor.config_kF(0, Constants.Values.DRIVE_VELOCITY_GAINS.kF, Constants.Values.TIMEOUT_MS);

        driveMotor.setNeutralMode(NeutralMode.Brake);

        /* Setup current limiting
		 *   enabled, Limit(amp), trigger Threshold(amp), Trigger Threshold Time(s)
         */
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 2.0));
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 1.0));
    }

    /**
     * Sets the amount of drive ticks per inch.
     * <p>
     * The amount of ticks per inch can be calculated by driving the robot some
     * distance forwards (10 feet is usually good) and then dividing the average
     * module ticks by that distance.
     * <p>
     * The default value uses inches and should only be used for testing.
     *
     * @param driveTicksPerUnit the amount of drive ticks that occur per unit of
     *                          travel
     */
    public void setDriveTicksPerUnit(double driveTicksPerUnit) {
        this.wheelRevolutionsPerUnit = driveTicksPerUnit;
    }

    @Override
    public double readAngle() {
        double voltage = angleAbsoluteEncoder.getVoltage() - .03; // account for dead encoder spot
        double angle = (voltage / 3.22) * 2.0 * Math.PI + offsetAngle;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public double getEncoderVoltage() {
        return angleAbsoluteEncoder.getPosition();
    }

    public double getIntegratedEncoderAngle() {
        return Math.toDegrees(angleMotorEncoder.getPosition());
    }

    public void resetIntegratedEncoder() {
        angleMotorEncoder.setPosition(readAngle());
    }

    /**
     * @return units per second of the drive motor
     */
    @Override
    public double getCurrentVelocity() {
        return driveMotor.getSelectedSensorVelocity() / 60.0 / DRIVE_GEAR_RATIO / wheelRevolutionsPerUnit;
    }

    @Override
    public double readDistance() {
        return driveMotor.getSelectedSensorPosition() / wheelRevolutionsPerUnit;
    }

    /**
     * @param angle IN RADIANS
     */
    @Override
    public void setTargetAngle(double angle) {
        angleMotor.set(anglePIDController.calculate(readAngle(), angle));
    }

    @Override
    public void setDriveOutput(double output) {
        driveMotor.set(ControlMode.PercentOutput, output);
    }
}
