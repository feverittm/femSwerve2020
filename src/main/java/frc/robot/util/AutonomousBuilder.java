/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commandgroups.CG_PrepShoot;
import frc.robot.commands.C_AutoAim;
import frc.robot.commands.C_AutoDrive;
import frc.robot.commands.C_Intake;
import frc.robot.commands.C_Shoot;
import frc.robot.commands.C_StopShooter;
import frc.robot.drivers.Vision;
import frc.robot.subsystems.SS_Drivebase;
import frc.robot.subsystems.SS_Feeder;
import frc.robot.subsystems.SS_Intake;
import frc.robot.subsystems.SS_Shooter;

public class AutonomousBuilder {

    private SendableChooser startingPositionSelector;
    private SendableChooser movementSelector;
    private NetworkTableEntry shootDelayEntry;
    private NetworkTableEntry movementDelayEntry;
    private SS_Drivebase drivebase = SS_Drivebase.getInstance();
    private Vision vision = RobotContainer.getVision();
    
    public AutonomousBuilder() {        
        initStartingPositionSelector();
        initMovementSelector();

        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Selector");
        //Add widgets to shuffleboard
        autoTab.add("Starting Position", startingPositionSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 0);
        shootDelayEntry = autoTab.add("Shoot Delay", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        movementDelayEntry = autoTab.add("Movement Delay", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
        autoTab.add("Movement Strategy", movementSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 3)
            .withSize(1, 2);
    }

    private void initStartingPositionSelector() {
        startingPositionSelector = new SendableChooser<StartingPosition>();
        //Add options to the starting position selector
        startingPositionSelector.setDefaultOption("Middle", StartingPosition.MIDDLE);
        startingPositionSelector.addOption("Left", StartingPosition.LEFT);
        startingPositionSelector.addOption("Right", StartingPosition.RIGHT);
    }

    private void initMovementSelector() {
        movementSelector = new SendableChooser<MovementStrategy>();
         //Add options to the movement strategy selector
        movementSelector.setDefaultOption("Backward", MovementStrategy.BACKWARD);
        movementSelector.addOption("Forward", MovementStrategy.FORWARD);
        movementSelector.addOption("Trench", MovementStrategy.TRENCH);
        movementSelector.addOption("None", MovementStrategy.NONE);
    }

    /**
     * Create the routine for autonomous based on the selected options from the Shuffleboard
     * @return A command group for autonomous
     */
    public SequentialCommandGroup buildAutoRoutine() {
        SequentialCommandGroup autoRoutine = new SequentialCommandGroup();

        autoRoutine.addCommands(
            new InstantCommand(() -> drivebase.resetGyroAngle(Rotation2.fromDegrees(180)), drivebase),
            new InstantCommand(() -> vision.setLEDMode(Vision.LED_ON)),
            new ParallelCommandGroup(
                new CG_PrepShoot(),
                new C_AutoAim()
            ),
            new WaitCommand(shootDelayEntry.getDouble(0.0)),
            new C_Shoot(),
            new C_Shoot(),
            new C_Shoot(),
            new InstantCommand(() -> vision.setLEDMode(Vision.LED_OFF)),
            new C_StopShooter(),
            new WaitCommand(movementDelayEntry.getDouble(0.0))
        );

        if(movementSelector.getSelected().equals(MovementStrategy.BACKWARD)){
            autoRoutine.addCommands(new C_AutoDrive(new Vector2(-24.0, 0.0), .5, Math.toRadians(180), 1.0));
        } else if (movementSelector.getSelected().equals(MovementStrategy.FORWARD)) {
            autoRoutine.addCommands(new C_AutoDrive(new Vector2(24.0, 0.0), .5, Math.toRadians(180), 1.0));
        } else if(movementSelector.getSelected().equals(MovementStrategy.TRENCH)) {
            if(startingPositionSelector.getSelected().equals(StartingPosition.RIGHT)){
                autoRoutine.addCommands(
                        new ParallelCommandGroup(
                            new C_AutoDrive(new Vector2(-190, 0.0), .5, Math.toRadians(180), 1.0),
                            new C_Intake()
                        )
                );
            } else if (startingPositionSelector.getSelected().equals(StartingPosition.MIDDLE)) {
                autoRoutine.addCommands(
                        new C_AutoDrive(new Vector2(-80.0, 66.0), .7, Math.toRadians(180), 1.0),
                        new ParallelCommandGroup(
                            new C_AutoDrive(new Vector2(-100.0, 0.0), .5, Math.toRadians(180), 1.0),
                            new C_Intake()
                        )
                );
            } else if (startingPositionSelector.getSelected().equals(StartingPosition.LEFT)) {
                autoRoutine.addCommands(
                    //TODO: add drive to trench command
                    new ParallelCommandGroup(
                        new C_AutoDrive(new Vector2(-100.0,0.0), .5, Math.toRadians(180), 1.0),
                        new C_Intake()
                    )
                );
            }
        }
        return autoRoutine;
    }

    public StartingPosition getStartingPosition() {
        return (StartingPosition)startingPositionSelector.getSelected();
    }

    public enum StartingPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum MovementStrategy {
        FORWARD,
        BACKWARD,
        TRENCH,
        NONE
    }
}