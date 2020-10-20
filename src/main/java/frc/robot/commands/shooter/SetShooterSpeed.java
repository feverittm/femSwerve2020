/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Gains;

public class SetShooterSpeed extends CommandBase {
  private double mSpeed = 0.0;
  private boolean stop = false;

  /**
   * Creates a new SetShooterSpeed.
   */
  public SetShooterSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Stop", stop);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mSetpoint = SmartDashboard.getNumber("SetPoint", 0);
    if (mSpeed != mSetpoint) {
      ShooterSubsystem.getInstance().setRPM(mSetpoint);
      mSpeed = mSetpoint;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.getInstance().goodStop();

    Gains myGains = ShooterSubsystem.getInstance().getGains(); 
    System.out.println("Shooter: kP=" + myGains.kP + ", kI=" + myGains.kI + ", kD=" + myGains.kD + ", kFF=" + myGains.kF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    stop = SmartDashboard.getBoolean("Stop", false);
    return stop;
  }
}
