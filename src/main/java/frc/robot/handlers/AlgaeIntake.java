// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.S_AlgaeIntake;

public class AlgaeIntake extends SubsystemBase implements StateSubsystem {
  private AlgaeIntakeStates desiredState, currentState = AlgaeIntakeStates.IDLE;
  private S_AlgaeIntake algaeIntake = S_AlgaeIntake.getInstance();
  private static AlgaeIntake m_Instance;

  private AlgaeIntake() {}

  public static AlgaeIntake getInstance() {
    if(m_Instance == null) {
      m_Instance = new AlgaeIntake();
    }

    return m_Instance;
  }

  @Override
  public void setDesiredState(State state) {
    if(desiredState != state) {
      desiredState = (AlgaeIntakeStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        algaeIntake.stop();
        break;
      case BROKEN:
        algaeIntake.stop();
        break;
      case INTAKING:
        algaeIntake.set(AlgaeIntakeConstants.ALGAE_INTAKING_SPEED);
        break;
      case SCORING:
        algaeIntake.set(AlgaeIntakeConstants.ALGAE_SCORING_SPEED);
        break;
      
      default:
        break;
    }
  }

  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case INTAKING:
        break;
      case SCORING:
        break;

      default:
        break;
    }

    if(!algaeIntake.checkSubsystem()) {
      setDesiredState(AlgaeIntakeStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    update();
  }

  public AlgaeIntakeStates getState() {
    return currentState;
  }

  public enum AlgaeIntakeStates implements State {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}
