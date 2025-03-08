// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Spitter;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;
  
  private final Manager m_Manager = new Manager();
  private final CommandXboxController m_controller = new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT);

  public RobotContainer() {
    configureAutos();
    configureElastic();
    configureBindings();
  }

  private void configureElastic() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    ElasticUtil.putString("Manager State", () -> m_Manager.getState().toString());
    ElasticUtil.putString("Swerve State", () -> Swerve.getInstance().getState().toString());
    ElasticUtil.putString("Spitter State", () -> Spitter.getInstance().getState().toString());
    ElasticUtil.putString("Pivot State", () -> Pivot.getInstance().getState().toString());
    ElasticUtil.putString("Elevator State", () -> Elevator.getInstance().getState().toString());
    ElasticUtil.putString("Algae Intake State", () -> AlgaeIntake.getInstance().getState().toString());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto chooser", autoChooser);
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new InstantCommand(() -> m_Manager.update(), m_Manager));

    // Stops movement - Works
    new JoystickButton(OI.driverJoytick, 1)
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.LOCKED), m_Manager))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Intakes coral spitter, moving elevator to intaking position
    m_controller.y()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.INTAKING_CORAL)))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    // Runs coral spitter, moving elevator to level one
    m_controller.x()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_ONE)))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    m_controller.b()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_TWO)))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    // Intakes with algae intake, and moves pivot to intaking position
    m_controller.leftBumper()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.INTAKING_ALGAE), m_Manager))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Scores with algae intake, and moves pivot to scoring position
    m_controller.leftTrigger()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_ALGAE), m_Manager))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
  }

  private void configureAutos() {
    NamedCommands.registerCommand("scoreLvl1", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_ONE), m_Manager));
    NamedCommands.registerCommand("scoreLvl2", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_TWO), m_Manager));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
