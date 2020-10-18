/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private Solenoid climberBrake;
  private CANSparkMax climberMotor;
  private CANEncoder climberEncoder;
  private VictorSPX crawlerMotor;

  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(Constants.Ports.CLIMBER_MOTOR, MotorType.kBrushless);
    crawlerMotor = new VictorSPX(Constants.Ports.CRAWLER_MOTOR);
    climberBrake = new Solenoid(Constants.Ports.CLIMBER_BRAKE);

    crawlerMotor.configFactoryDefault(10);
    crawlerMotor.setNeutralMode(NeutralMode.Brake);
    crawlerMotor.setInverted(false);

    climberEncoder = climberMotor.getEncoder();

    climberMotor.setSmartCurrentLimit(70);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void extendPiston(){
    climberBrake.set(false);
  }

  public void retractPiston(){
    climberBrake.set(true);
  }

  public void setMotorBrake() {
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setMotorCoast() {
    climberMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setMotorSpeed(double speed){
    if (!climberBrake.get()) {
      climberBrake.set(true);
    }
    climberMotor.set(speed);
  }

  public void setCrawlSpeed(double speed){
    crawlerMotor.set(ControlMode.PercentOutput, speed);
  }

  /*
   * number of encoder tics on the Neo
   */
  public double getEncoder() {
    return climberEncoder.getPosition();
  }

  /*
   * return the height of the climber
   */
  public double getHeight() {
    return getEncoder() * Constants.Values.CLIMBER_HEIGHT_REVS_TO_HEIGHT + Constants.Values.CLIMBER_HEIGHT_OFFSET;
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber/Encoder hright", getHeight());
  }

  private static ClimberSubsystem instance;
  public static ClimberSubsystem getInstance(){
    if (instance == null){
      instance = new ClimberSubsystem();
    }
    return instance;
  }
}
