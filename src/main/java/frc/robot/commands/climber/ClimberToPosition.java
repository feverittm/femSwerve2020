/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ClimberToPosition extends PIDCommand {
  /**
   * Creates a new ClimberToPosition.
   */
  public ClimberToPosition(double position, ClimberSubsystem climber) {
    super(
        // The controller that the command will use
        new PIDController(Constants.Values.CLIMBER_P, Constants.Values.CLIMBER_I, Constants.Values.CLIMBER_D),
        // Close loop on the elevator height
        climber::getHeight,
        // This should return the setpoint (can also be a constant)
        position,
        // This uses the output
        output -> climber.setMotorSpeed(output));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ClimberSubsystem.getInstance());
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10, 10);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
