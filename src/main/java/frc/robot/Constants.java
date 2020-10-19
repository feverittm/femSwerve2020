/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.Gains;

public final class Constants {

  public static boolean eps(double a, double b, double eps) {
    return Math.abs(a - b) < eps;
  }

  public static class Ports {

    public static final int SHOOTER_MOTOR_1 = 8; // CAN
    public static final int SHOOTER_MOTOR_2 = 9; // CAN
    public static final int SHOOTER_IR = 1; // DIO
    public static final int OVERFLOW_IR = 2; // DIO
 
    public static final int INTAKE_MOTOR_2 = 11;
    public static final int INTAKE_SOLENOID = 3;
    public static final int INTAKE_MOTOR_1 = 7; 
    public static final int INTAKE_IR = 0;

    // Hopper things
    public static final int HOPPER_MOTOR_TOP = 6;
    public static final int HOPPER_MOTOR_BOTTOM = 5;

    // Drivetrain things
    public static final int ULTRASONIC_CHANNEL = 4;

    // climber things
    public static final int CLIMBER_MOTOR = 10;
    public static final int CRAWLER_MOTOR = 11;
    public static final int CLIMBER_BRAKE = 16;

    // Swerve Drive Module configuration
    protected static final int[] AZIMUTH_PORTS = { 12, 13, 14, 15 };
    protected static final int[] DRIVE_PORTS = { 1, 2, 3, 4 };
    protected static final int[] MODULE_ENCODERS = { 0, 1, 2, 3 };
  }

  public static class Values {
    public static final double WHEEL_BASE = 22.0;
    public static final double TRACK_WIDTH = 12.0;
    public static final double VOLTAGE_TO_FEET = (12 * 0.0098); // 9.8mV per inch with a 5V input. For ultrasonic.
    public static final double VISION_ANGLE_TOLERANCE = 1.5;
    public static final double VISION_DRIVE_P = 0.000025;
    public static final double VISION_DRIVE_I = 0.00006; 
    public static final double VISION_DRIVE_D = 0.00003;
    public static final double ACCELERATION = 2.5; // Percentage / Seconds
    public static final double VISION_TURNING_P = 0.025; // 0.04
    public static final double VISION_TURNING_I = 0.06;
    public static final double VISION_TURNING_D = 0.07;
    public static final double VISION_TOLERANCE = 1.5;
    public static final double VISION_TIMEOUT = 2000; // in ms
    public static final double VISION_LIMELIGHT_HEIGHT = 40; // Height (inches) up from the ground of the center of the limelight.
    public static final double VISION_LIMELIGHT_ANGLE = 30; // Math.atan(2.5/1.75) * (180 / Math.PI), //angle the limelight is tilted at. In
                                                            // degrees up from the floor.
    public static final double INTAKE_IN = 0.6; // 0.75 // percent speed to intake
    public static final double INTAKE_EJECT = -0.5; // percent speed to outtake
    public static final double INTAKE_EXTEND_DELAY = 0.2; // seconds
    public static final double HOPPER_HANDOFF_DELAY = 0.0; // 0.13 * (speed / 0.75)
    public static final double HOPPER_HANDOFF_ROLL_TIME = 0.68;
    public static final double HOPPER_INTAKE_SPEED = 0.4; 
    public static final double HOPPER_STREAM_SPEED = 1; 
    public static final double HOPPER_EJECT_SPEED = -0.4;
    public static final double HOPPER_INTAKE_IR_DELAY = 0; // ms
    public static final double HOPPER_SHOOTER_IR_DELAY = 20; // ms
    public static final double SHOOTER_GEARING = 9.0 / 11.0;
    public static final double SHOOTER_MAX_RPM = 4800; // adjusted for gearing - raw ~5000 rpm
    public static final double SHOOTER_RPM = 3700;
    public static final double SHOOTER_CIRCUMFERENCE_CM = (10.16 * Math.PI); // cm
    public static final double SHOOTER_RELEASE_ANGLE = 80; // degrees up from horizontal
    public static final double SHOOTER_RELEASE_HEIGHT = 1.0612121212; // meters up from ground
    public static final double CLIMBER_UP = 1;
    public static final double CLIMBER_DOWN = -1;
    public static final double CLIMBER_P = 0; 
    public static final double CLIMBER_I = 0; 
    public static final double CLIMBER_D = 0;
    public static final double CLIMBER_HEIGHT_REVS_TO_HEIGHT = 1; 
    public static final double CLIMBER_HEIGHT_OFFSET = 25; // inches
    
    // To Seconds, To RPM Motorside, To RPM Wheelside, To Circumference, To Feet
    public static final double DRIVE_VEL_2_FEET = 10 * (1.0 / 2048.0) * (9.0 / 70.0) * (4.875 * Math.PI) * (1.0 / 12.0);

    protected static final double[] MODULE_ZEROS = { 0.0, 0.0, 0.0, 0.0 };

    public static final Gains DRIVE_VELOCITY_GAINS = new Gains(0.6, 0.012, 6.8, 1023.0 / 21500.0);
    public static final Gains SHOOTER_VELOCITY_GAINS = new Gains(0.001, 0, 0.005, (1.0 / (4060.0 * (22.0 / 18.0) * 0.5)));
    public static final Gains REV_SHOOTER_VELOCITY_GAINS = new Gains(5e-5, 1e-6, 0.0, 0.000156);

    protected static final Gains[] DRIVE_GAINS = { 
      new Gains(0.0, 0.0, 0.0, 0.0), 
      new Gains(0.0, 0.0, 0.0, 0.0), 
      new Gains(0.0, 0.0, 0.0, 0.0),
      new Gains(0.0, 0.0, 0.0, 0.0) 
    };

    protected static final Gains[] AZIMUTH_GAINS = { 
      new Gains(0.01, 0.0, 0.0, 0.0),
      new Gains(0.01, 0.0, 0.0, 0.0),
      new Gains(0.01, 0.0, 0.0, 0.0),
      new Gains(0.01, 0.0, 0.0, 0.0) 
    };

    public static final int LED_COUNT = 35;
    public static final int LED_WIDTH = 7; 
    public static final int LED_ROWS = 5;
  }
}
