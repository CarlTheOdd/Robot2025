// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.handlers.AlgaeIntake;
import frc.robot.handlers.Elevator;
import frc.robot.handlers.Manager;
import frc.robot.handlers.Manager.ManagerStates;
import frc.robot.handlers.Pivot;
import frc.robot.handlers.Spitter;
import frc.robot.handlers.Swerve;
import frc.robot.handlers.Swerve.SwerveStates;
import frc.robot.subsystems.S_Swerve;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureAutos();
    configureElastic();
    configureBindings();
  }

  private void configureElastic() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    ElasticUtil.putString("Manager State", () -> Manager.getInstance().getState().toString());
    ElasticUtil.putString("Swerve State", () -> Swerve.getInstance().getState().toString());
    ElasticUtil.putString("Spitter State", () -> Spitter.getInstance().getState().toString());
    ElasticUtil.putString("Pivot State", () -> Pivot.getInstance().getState().toString());
    ElasticUtil.putString("Elevator State", () -> Elevator.getInstance().getState().toString());
    ElasticUtil.putString("Algae Intake State", () -> AlgaeIntake.getInstance().getState().toString());

    autoChooser = new SendableChooser<Command>();

    RunCommand leave = new RunCommand(() -> S_Swerve.getInstance().drive(-0.5, 0, -S_Swerve.getInstance().angleController.calculate(S_Swerve.getInstance().getHeading(), 180), false), Swerve.getInstance());
    
    autoChooser.addOption("leave thing", leave);

    SmartDashboard.putData("auto chooser", autoChooser);
  }

  private void configureBindings() {
    // Lifts elevator to knock off algae, and runs rollers
    Manager.getInstance().bindState(OI.auxController.a(), ManagerStates.KNOCKING_ALGAE, ManagerStates.DRIVE);

    // Intakes with algae intake, and moves pivot to intaking position
    Manager.getInstance().bindState(OI.auxController.leftBumper(), ManagerStates.INTAKING_ALGAE, ManagerStates.DRIVE);

    // Scores with algae intake, and moves pivot to scoring position
    Manager.getInstance().bindState(OI.auxController.rightBumper(), ManagerStates.SCORING_ALGAE, ManagerStates.DRIVE);

    // Sets the wheels to an X formation
    Swerve.getInstance().bindState(OI.driverController.rightBumper(), SwerveStates.LOCKED, SwerveStates.DRIVE);

    // Zeroes the gyro
    OI.driverController.start()
      .onTrue(new InstantCommand(() -> S_Swerve.getInstance().setHeading(0), Swerve.getInstance()));

    // Shuts off normal driving and drives to apriltag (untested)
    // OI.driverController.a()
    //   .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.AIMING), Swerve.getInstance()))
    //   .onFalse(new InstantCommand(() -> {
    //     Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
    //     Swerve.getInstance().setTracking(false);
    //   }, Swerve.getInstance()));
  }

  private void configureAutos() {
    NamedCommands.registerCommand("scoreAlgae", new InstantCommand(() -> Manager.getInstance().setDesiredState(ManagerStates.SCORING_ALGAE), Manager.getInstance()));
    NamedCommands.registerCommand("drive", new InstantCommand(() -> Manager.getInstance().setDesiredState(ManagerStates.DRIVE), Manager.getInstance()));
    NamedCommands.registerCommand("idle", new InstantCommand(() -> Manager.getInstance().setDesiredState(ManagerStates.IDLE), Manager.getInstance()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
