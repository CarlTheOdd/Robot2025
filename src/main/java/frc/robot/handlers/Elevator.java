// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.S_Elevator;

public class Elevator extends SubsystemBase implements StateSubsystem {
  private ElevatorStates desiredState, currentState = ElevatorStates.IDLE;
  private S_Elevator elevator = S_Elevator.getInstance();
  private static Elevator m_Instance;

  private Elevator() {}

  public static Elevator getInstance() {
    if(m_Instance == null) {
      m_Instance = new Elevator();
    }

    return m_Instance;
  }

  @Override
  public void setDesiredState(State state) {
    if(desiredState != state) {
      desiredState = (ElevatorStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        elevator.stop();
        break;
      case BROKEN:
        elevator.stop();
        break;
      case HOME:
        elevator.setSetpoint(ElevatorConstants.HOME_POSITION);
        break;
      case L2:
        elevator.setSetpoint(ElevatorConstants.L2_POSITION);
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case HOME:
        elevator.moveToSetpoint();
        break;
      case L2:
        elevator.moveToSetpoint();
        break;
      
      default:
        break;
    }

    if(!elevator.checkSubsystem()) {
      setDesiredState(ElevatorStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    update();
  }

  public ElevatorStates getState() {
    return currentState;
  }

  public enum ElevatorStates implements State {
    IDLE,
    BROKEN,
    HOME,
    L2;
  }
}
