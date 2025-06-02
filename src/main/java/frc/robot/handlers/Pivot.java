// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.S_Pivot;

public class Pivot extends SubsystemBase implements StateSubsystem {
  private PivotStates desiredState, currentState = PivotStates.IDLE;
  private S_Pivot pivot = S_Pivot.getInstance();
  private static Pivot m_Instance;

  private Pivot() {}

  public static Pivot getInstance() {
    if(m_Instance == null) {
      m_Instance = new Pivot();
    }

    return m_Instance;
  }

  @Override
  public void setDesiredState(State state) {
    if(desiredState != state) {
      desiredState = (PivotStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        pivot.stop();
        break;
      case BROKEN:
        pivot.stop();
        break;
      case STORED:
        pivot.setSetpoint(PivotConstants.HOME_ROTATION);
        break;
      case SCORING:
        pivot.setSetpoint(PivotConstants.SCORE_ROTATION);
        break;
      case INTAKING:
        pivot.setSetpoint(PivotConstants.INTAKE_ROTATION);
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
      case STORED:
        pivot.moveToSetpoint();
        break;
      case SCORING:
        pivot.moveToSetpoint();
        break;
      case INTAKING:
        pivot.moveToSetpoint();
        break;
      
      default:
        break;
    }

    if(!pivot.checkSubsystem()) {
      setDesiredState(PivotStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    update();
  }

  public PivotStates getState() {
    return currentState;
  }
  
  public enum PivotStates implements State {
    IDLE,
    BROKEN,
    STORED,
    SCORING,
    INTAKING;
  }
}
