/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

/**
  * Creates a new ClimberMove.
  */

public class ClimberMove extends CommandBase {
  private final ClimberSubsystem m_climber;
  private final double mSpeed;

  /*
   * @speed speed to move the climber
   * @climber the climber to use
   */

  public ClimberMove(double speed, ClimberSubsystem climber) {
    mSpeed = speed;
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.retractPiston();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setMotorSpeed(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimberSubsystem.getInstance().setMotorSpeed(0.0);
    ClimberSubsystem.getInstance().extendPiston();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
