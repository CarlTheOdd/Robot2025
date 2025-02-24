// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PivotConstants;
import frc.utils.Utils;

// This is for the algae
public class Pivot extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private static Pivot m_instance;

  private PIDController angleController;

  private PivotStates desiredState = PivotStates.IDLE, currentState = PivotStates.IDLE;

  /** Creates a new Pivot. */
  public Pivot() {
    motor = new SparkMax(CANConstants.PIVOT_ID, MotorType.kBrushless);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_NEO)
      .encoder.positionConversionFactor(PivotConstants.PIVOT_MOTOR_REDUCTION);

    angleController = new PIDController(0.01, 0, 0);
    angleController.setTolerance(3);

    initialized = true;
  }

  public static Pivot getInstance() {
    if(m_instance == null) {
      m_instance = new Pivot();
    }

    return m_instance;
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
    status &= getInitialized();

    return status;
  }

  @Override
  public void periodic() {}

  public void update() {
    switch(currentState) {
      case IDLE:
        setDesiredState(PivotStates.STORED);
        break;
      case BROKEN:
        break;
      case STORED:
      case SCORING:
      case INTAKING:
        motor.set(Utils.normalize(angleController.calculate(motor.getEncoder().getPosition())));
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(PivotStates.BROKEN);
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
      case BROKEN:
        stop();
        break;
      case STORED:
        angleController.setSetpoint(PivotConstants.HOME_ROTATION);
        break;
      case SCORING:
        angleController.setSetpoint(PivotConstants.SCORE_ROTATION);
        break;
      case INTAKING:
        angleController.setSetpoint(PivotConstants.INTAKE_ROTATION);
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  public void setDesiredState(PivotStates state) {
    if(this.desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  public PivotStates getState() {
    return currentState;
  }

  public enum PivotStates {
    IDLE,
    BROKEN,
    STORED,
    SCORING,
    INTAKING;
  }
}
