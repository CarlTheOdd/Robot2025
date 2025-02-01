// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PivotConstants;

// This is for the algae
public class Pivot extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private static Pivot m_instance;

  private PIDController pid = new PIDController(0.01, 0, 0);

  private PivotStates desiredState, currentState = PivotStates.IDLE;

  /** Creates a new Pivot. */
  public Pivot() {
    motor = new SparkMax(CANConstants.PIVOT_ID, MotorType.kBrushless);

    initialized = true;
  }

  public static Pivot getInstance() {
    if(m_instance == null) {
      m_instance = new Pivot();
    }

    return m_instance;
  }

  public void store() {
    pid.setSetpoint(PivotConstants.HOME_ROTATION);

    if(!pid.atSetpoint()) {
      motor.set(pid.calculate(motor.getEncoder().getPosition()));
    }
  }

  public void score() {
    pid.setSetpoint(PivotConstants.SCORE_ROTATION);

    if(!pid.atSetpoint()) {
      motor.set(pid.calculate(motor.getEncoder().getPosition()));
    }
  }

  public void intake() {
    pid.setSetpoint(PivotConstants.INTAKE_ROTATION);

    if(!pid.atSetpoint()) {
      motor.set(pid.calculate(motor.getEncoder().getPosition()));
    }
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
        break;
      case BROKEN:
        break;
      case STORED:
        store();
        break;
      case SCORING:
        score();
        break;
      case INTAKING:
        intake();
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
        stop();
        break;
      case BROKEN:
        stop();
        break;
      case STORED:
        break;
      case SCORING:
        break;
      case INTAKING:
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

  public enum PivotStates {
    IDLE,
    BROKEN,
    STORED,
    SCORING,
    INTAKING;
  }
}
