// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.CANConstants;

public class AlgaeIntake extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status = false, initialized = false;
  
  private SparkMax motor;

  private static AlgaeIntake m_instance;

  private AlgaeIntakeStates currentState = AlgaeIntakeStates.IDLE, desiredState = AlgaeIntakeStates.IDLE;

  public AlgaeIntake() {
    motor = new SparkMax(CANConstants.ALGAE_INTAKE_ID, MotorType.kBrushless);

    SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
    
    algaeIntakeConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_550);
    
    motor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    initialized = true;
  }

  public static AlgaeIntake getInstance() {
    if(m_instance == null) {
      m_instance = new AlgaeIntake();
    }

    return m_instance;
  }

  @Override
  public void periodic() {}

  public void setSpeed(double speed) {
    motor.set(speed);
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
    status &= initialized;

    return status;
  }

  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
      case BROKEN:
        break;
      case INTAKING:
        setSpeed(AlgaeIntakeConstants.ALGAE_INTAKING_SPEED);
        break;
      case SCORING:
        setSpeed(AlgaeIntakeConstants.ALGAE_SCORING_SPEED);
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
      case BROKEN:
        stop();
        break;
      case INTAKING:
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

  public AlgaeIntakeStates getState() {
    return currentState;
  }

  public enum AlgaeIntakeStates {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}
