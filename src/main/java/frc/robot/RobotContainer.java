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
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeStates;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Pivot.PivotStates;
import frc.robot.subsystems.Roller.RollerStates;

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
    main.addString("Swerve State", () -> m_Manager.swerve.getState().toString());
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));

    // Stops movement - Works
    new JoystickButton(OI.driverJoytick, 1)
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.LOCKED), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Runs rollers
    m_controller.x().whileTrue(new RunCommand(
      () -> m_Manager.roller.setDesiredState(RollerStates.RUNNING),
      m_Manager.roller
    ));

    // Intakes with algae intake
    m_controller.leftBumper().whileTrue(new RunCommand(
      () -> m_Manager.algaeIntake.setDesiredState(AlgaeIntakeStates.INTAKING),
      m_Manager.algaeIntake,
      m_Manager.pivot
    ));

    // Scores with algae intake
    m_controller.leftTrigger().whileTrue(new RunCommand(
      () -> m_Manager.algaeIntake.setDesiredState(AlgaeIntakeStates.SCORING),
      m_Manager.algaeIntake,
      m_Manager.pivot
    ));

    // Sets algae intake to home position
    m_controller.a().whileTrue(new RunCommand(
      () -> m_Manager.pivot.setDesiredState(PivotStates.STORED),
      m_Manager.pivot,
      m_Manager.algaeIntake
    ));

    // Sets algae intake to scoring position
    m_controller.y().whileTrue(new RunCommand(
      () -> m_Manager.pivot.setDesiredState(PivotStates.SCORING),
      m_Manager.pivot,
      m_Manager.algaeIntake
    ));

    // Sets algae intake to intaking position
    m_controller.b().whileTrue(new RunCommand(
      () -> m_Manager.pivot.setDesiredState(PivotStates.INTAKING),
      m_Manager.pivot,
      m_Manager.algaeIntake
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
