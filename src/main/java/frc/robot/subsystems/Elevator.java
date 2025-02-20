// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean initialized = false, status = false;

  private SparkMax motor1, motor2;

  private static Elevator m_instance;

  private PIDController pid;

  private ElevatorStates currentState = ElevatorStates.IDLE, desiredState = ElevatorStates.IDLE;

  public Elevator() {
    motor1 = new SparkMax(CANConstants.ELEVATOR_MOTOR_ONE_ID, MotorType.kBrushless);
    motor2 = new SparkMax(CANConstants.ELEVATOR_MOTOR_TWO_ID, MotorType.kBrushless);

    pid = new PIDController(0.05, 0, 0);
    pid.setTolerance(3);

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

  public void setSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  public void goToHomePos() {
    pid.setSetpoint(ElevatorConstants.HOME_POSITION);

    if(!pid.atSetpoint()) {
      setSpeed(pid.calculate(motor1.getEncoder().getPosition()));
    }
  }

  public void goToL2Pos() {
    double setpoint = (ElevatorConstants.L2_POSITION / ElevatorConstants.ELEVATOR_PULLY_CIRCUMFERENCE) * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    pid.setSetpoint(setpoint);

    if(!pid.atSetpoint()) {
      setSpeed(pid.calculate(motor1.getEncoder().getPosition()));
    }
  }

  public void goToClimbPos() {
    double setpoint = (ElevatorConstants.CLIMB_POSITION / ElevatorConstants.ELEVATOR_PULLY_CIRCUMFERENCE) * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    pid.setSetpoint(setpoint);
    
    if(!pid.atSetpoint()) {
      setSpeed(pid.calculate(motor1.getEncoder().getPosition()));
    }
  }

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
        goToHomePos();
        break;
      case L2:
        goToL2Pos();
        break;
      case CLIMBING:
        goToClimbPos();
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

  public ElevatorStates getState() {
    return currentState;
  }

  public enum ElevatorStates {
    IDLE,
    BROKEN,
    HOME,
    L2,
    CLIMBING;
  }
}
