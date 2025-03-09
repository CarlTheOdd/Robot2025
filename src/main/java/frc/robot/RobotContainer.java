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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Spitter;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveStates;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;
  
  private final Manager m_Manager = new Manager();

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
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));
    Swerve.getInstance().setDefaultCommand(new RunCommand(() -> Swerve.getInstance().update(), Swerve.getInstance()));

    // Stops movement - Works
    // new JoystickButton(OI.driverJoytick, 1)
    //   .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.LOCKED), m_Manager))
    //   .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Intakes coral spitter, moving elevator to intaking position
    OI.auxController.x()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.INTAKING_CORAL)))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    // Runs coral spitter, moving elevator to level one
    OI.auxController.y()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_ONE)))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    OI.auxController.b()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_TWO)))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE)));

    // Intakes with algae intake, and moves pivot to intaking position
    OI.auxController.leftBumper()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.INTAKING_ALGAE), m_Manager))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Scores with algae intake, and moves pivot to scoring position
    OI.auxController.rightBumper()
      .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_ALGAE), m_Manager))
      .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Sets the wheels to an X formation
    OI.driverController.rightBumper()
      .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.LOCKED), Swerve.getInstance()))
      .onFalse(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.DRIVE), Swerve.getInstance()));

    // Shuts off normal driving and drives to apriltag (untested)
    OI.driverController.a()
      .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.AIMING), Swerve.getInstance()))
      .onFalse(new InstantCommand(() -> {
        Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
        Swerve.getInstance().setTracking(false);
      }, Swerve.getInstance()));
  }

  private void configureAutos() {
    NamedCommands.registerCommand("scoreLvl1", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_ONE), m_Manager));
    NamedCommands.registerCommand("scoreLvl2", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_LEVEL_TWO), m_Manager));
    NamedCommands.registerCommand("scoreAlgae", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.SCORING_ALGAE), m_Manager));
    NamedCommands.registerCommand("drive", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    NamedCommands.registerCommand("idle", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.IDLE), m_Manager));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
