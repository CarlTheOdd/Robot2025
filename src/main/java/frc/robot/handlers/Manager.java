// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.handlers.AlgaeIntake.AlgaeIntakeStates;
import frc.robot.handlers.Elevator.ElevatorStates;
import frc.robot.handlers.Pivot.PivotStates;
import frc.robot.handlers.Spitter.SpitterStates;
import frc.robot.subsystems.S_AlgaeIntake;
import frc.robot.subsystems.S_Elevator;
import frc.robot.subsystems.S_Pivot;
import frc.robot.subsystems.S_Spitter;

public class Manager extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  // private Swerve swerve = Swerve.getInstance();
  private S_Spitter spitter = S_Spitter.getInstance();
  private S_Pivot pivot = S_Pivot.getInstance();
  private S_AlgaeIntake algaeIntake = S_AlgaeIntake.getInstance();
  private S_Elevator elevator = S_Elevator.getInstance();

  private static Manager m_Instance;
  private ManagerStates desiredState, currentState = ManagerStates.IDLE;

  /** Creates a new Manager. */
  private Manager() {
    // All subsystems should initialize when calling getInstance()
    initialized &= spitter.getInitialized();
    initialized &= pivot.getInitialized();
    initialized &= algaeIntake.getInitialized();
    initialized &= elevator.getInitialized();
  }

  public static Manager getInstance() {
    if(m_Instance == null) {
      m_Instance = new Manager();
    }

    return m_Instance;
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    spitter.stop();
    pivot.stop();
    algaeIntake.stop();
    elevator.stop();
  }

  /**
   * @return Has the constructor executed successfully
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }

  /**
   * @return Is the robot is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status &= spitter.checkSubsystem();
    status &= pivot.checkSubsystem();
    status &= algaeIntake.checkSubsystem();
    status &= elevator.checkSubsystem();

    return status;
  }

   /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        // The robot should never be IDLE in a match
        setDesiredState(ManagerStates.DRIVE);
        break;
      case DRIVE:
      case LOCKED:
      case KNOCKING_ALGAE:
      case INTAKING_ALGAE:
      case SCORING_ALGAE:
        break;

      default:
        break;
    }
  }

  /**
   * Handles moving from one state to another. Also changes
   * the state of all subsystems to their respective states
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        Spitter.getInstance().setDesiredState(SpitterStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.IDLE);
        AlgaeIntake.getInstance().setDesiredState(AlgaeIntakeStates.IDLE);
        Elevator.getInstance().setDesiredState(ElevatorStates.IDLE);
        break;
      case DRIVE:
        Spitter.getInstance().setDesiredState(SpitterStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeIntakeStates.IDLE);
        Elevator.getInstance().setDesiredState(ElevatorStates.HOME);
        break;
      case LOCKED:
        Spitter.getInstance().setDesiredState(SpitterStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeIntakeStates.IDLE);
        Elevator.getInstance().setDesiredState(ElevatorStates.HOME);
        break;
      case KNOCKING_ALGAE:
        Spitter.getInstance().setDesiredState(SpitterStates.RUNNING);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeIntakeStates.IDLE);
        Elevator.getInstance().setDesiredState(ElevatorStates.L2);
      case INTAKING_ALGAE:
        Spitter.getInstance().setDesiredState(SpitterStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.INTAKING);
        AlgaeIntake.getInstance().setDesiredState(AlgaeIntakeStates.INTAKING);
        Elevator.getInstance().setDesiredState(ElevatorStates.HOME);
        break;
      case SCORING_ALGAE:
        Spitter.getInstance().setDesiredState(SpitterStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.SCORING);
        AlgaeIntake.getInstance().setDesiredState(AlgaeIntakeStates.SCORING);
        Elevator.getInstance().setDesiredState(ElevatorStates.HOME);
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  /**
   * Sets the desired state of the subsystem
   * @param state Desired state
   */
  @Override
  public void setDesiredState(State state) {
    if(desiredState != state) {
      desiredState = (ManagerStates) state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public ManagerStates getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    update();
  }

  public Trigger bindState(Trigger button, ManagerStates onTrue, ManagerStates onFalse) {
    return button
      .onTrue(new InstantCommand(() -> setDesiredState(onTrue), this))
      .onFalse(new InstantCommand(() -> setDesiredState(onFalse), this));
  }

  /**
   * A Manager state integrates all of the other subsystem's states together
   * to create one cohesive state. This ensures that every subsystem is in
   * the correct state when performing an action. For example, when the
   * pivot is aiming, the shooter should be spinning up. When setting
   * the pivot state you may forget to spin up the shooter.
   *
   * <p>The "BROKEN" state which is present in all other subsystems
   * is not present here because it would mean that the whole
   * robot is broken. In that scenario the robot would be E-stopped
   * anyway and no code would run be running.
  */
  public enum ManagerStates implements State {
    IDLE,
    /** Just driving the robot around */
    DRIVE,
    /** Locking the wheels in an X formation */
    LOCKED,
    /** Knocking algae out of the reef */
    KNOCKING_ALGAE,
    /** Intaking with the algae intake, also sets pivot position */
    INTAKING_ALGAE,
    /** Scoring with the algae intake, also sets pivot position */
    SCORING_ALGAE;
  }
}
