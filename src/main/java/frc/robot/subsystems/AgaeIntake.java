// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class AgaeIntake extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status, initialized = false;
  
  private SparkMax motor;

  private AlgaeIntakeStates currentState, desiredState = AlgaeIntakeStates.IDLE;

  public AgaeIntake() {
    motor = new SparkMax(CANConstants.ALGAE_INTAKE_ID, MotorType.kBrushless);

    initialized = true;
  }

  @Override
  public void periodic() {}

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public boolean getInitialized() {
    return initialized;
  }

  public boolean checkSubsystem() {
    status &= initialized;

    return status;
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
    
    if(!checkSubsystem()) {
      setDesiredState(AlgaeIntakeStates.BROKEN);
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        stop();
        break;
      case BROKEN:
        stop();
        break;
      case INTAKING:
        break;
      case SCORING:
        break;
        
      default:
        break;
    }

    currentState = desiredState;
  }

  public void setDesiredState(AlgaeIntakeStates state) {
    if(desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  public enum AlgaeIntakeStates {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}
