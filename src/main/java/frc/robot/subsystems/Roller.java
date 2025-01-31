// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.RollerConstants;

// This is for the coral
public class Roller extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private static Roller m_instance;

  private RollerStates desiredState, currentState = RollerStates.IDLE;

  /** Creates a new Rollers. */
  public Roller() {
    motor = new SparkMax(CANConstants.ROLLER_ID, MotorType.kBrushless);

    initialized = true;
    status = true;
  }
  
  public static Roller getInstance() {
    if(m_instance == null) {
      m_instance = new Roller();
    }

    return m_instance;
  }

  public void runRollers() {
    motor.set(RollerConstants.ROLLER_SPEED);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public boolean getInitialized() {
    return initialized;
  }

  @Override
  public boolean checkSubsystem() {
    status = getInitialized();

    return status;
  }

  @Override
  public void periodic() {}

  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case RUNNING:
        runRollers();
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(RollerStates.BROKEN);
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
      case RUNNING:
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  public void setDesiredState(RollerStates state) {
    if(this.desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  public enum RollerStates {
    IDLE,
    BROKEN,
    RUNNING;
  }
}
