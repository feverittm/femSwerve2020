/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

  private ClimberSubsystem() {
    climberMotor = new CANSparkMax(Constants.Ports.climberMotorPort, MotorType.kBrushless);
    climberBrake = new Solenoid(Constants.Ports.climberBrakePort);

    climberEncoder = climberMotor.getEncoder();

    climberMotor.setSmartCurrentLimit(70);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void ExtendPiston(){
    climberBrake.set(false);
  }

  public void RetractPiston(){
    climberBrake.set(true);
  }

  public void setBrake() {
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    climberMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setSpeed(double speed){
    climberMotor.set(speed);
  }

  public double getEncoder() {
    return climberEncoder.getPosition();
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber/Encoder value", getEncoder());
  }

  public static ClimberSubsystem instance;
  public static ClimberSubsystem getInstance(){
    if (instance == null){
      instance = new ClimberSubsystem();
    }
    return instance;
  }
}
