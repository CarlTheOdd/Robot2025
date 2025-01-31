// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Elevator extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean initialized, status = false;

  private SparkMax motor1, motor2;

  private static Elevator m_instance;

  private ElevatorStates currentState, desiredState = ElevatorStates.IDLE;

  public Elevator() {
    motor1 = new SparkMax(CANConstants.ELEVATOR_MOTOR_ONE_ID, MotorType.kBrushless);
    motor2 = new SparkMax(CANConstants.ELEVATOR_MOTOR_TWO_ID, MotorType.kBrushless);

    initialized = true;
  }

  public static Elevator getInstance() {
    if(m_instance == null) {
      m_instance = new Elevator();
    }

    return m_instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  @Override
  public boolean getInitialized() {
    return initialized;
  }

  @Override
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
      case HOME:
        break;
      case L2:
        break;
      case CLIMBING:
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(ElevatorStates.BROKEN);
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
      case HOME:
        break;
      case L2:
        break;
      case CLIMBING:
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  public void setDesiredState(ElevatorStates state) {
    if(desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  public enum ElevatorStates {
    IDLE,
    BROKEN,
    HOME,
    L2,
    CLIMBING;
  }
}
