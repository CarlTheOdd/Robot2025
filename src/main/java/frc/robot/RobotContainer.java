// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Spitter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  private final Manager m_Manager = new Manager();
  private final CommandXboxController m_controller = new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT);

  public RobotContainer() {
    // This would throw an error no matter what
    // in last year's code, so let's hope for better
    // this year.
    autoChooser = AutoBuilder.buildAutoChooser();

    configureElastic();
    configureBindings();
  }

  private void configureElastic() {
    // The main tab is used during a match to display relavent information
    ShuffleboardTab main = Shuffleboard.getTab("Main");
    main.addString("Manager State", () -> m_Manager.getState().toString());
    main.addString("Swerve State", () -> Swerve.getInstance().getState().toString());
    main.addString("Roller State", () -> Spitter.getInstance().getState().toString());
    main.addString("Pivot State", () -> Pivot.getInstance().getState().toString());
    main.addString("Algae Intake State", () -> AlgaeIntake.getInstance().getState().toString());
    main.addString("Elevator State", () -> Elevator.getInstance().getState().toString());
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));

    // Stops movement - Works
    new JoystickButton(OI.driverJoytick, 1)
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.LOCKED), m_Manager))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Intakes coral spitter, moving elevator to intaking position
    m_controller.y()
      .onTrue(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.INTAKING_CORAL)))
      .onFalse(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    // Runs coral spitter, moving elevator to level one
    m_controller.x()
      .onTrue(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_ONE)))
      .onFalse(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    m_controller.b()
      .onTrue(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_TWO)))
      .onFalse(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    // Intakes with algae intake, and moves pivot to intaking position
    m_controller.leftBumper()
      .onTrue(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.INTAKING_ALGAE), m_Manager))
      .onFalse(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Scores with algae intake, and moves pivot to scoring position
    m_controller.leftTrigger()
      .onTrue(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_ALGAE), m_Manager))
      .onFalse(new RunCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
