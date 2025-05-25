// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

public class Elevator extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean initialized = false, status = false;

  private SparkMax motor1, motor2;

  private static Elevator m_instance;

  private PIDController posController;
  private double p = 2, i = 0, d = 0;

  private ElevatorStates currentState = ElevatorStates.IDLE, desiredState = ElevatorStates.IDLE;

  public Elevator() {
    motor1 = new SparkMax(CANConstants.ELEVATOR_MOTOR_ONE_ID, MotorType.kBrushless);
    motor2 = new SparkMax(CANConstants.ELEVATOR_MOTOR_TWO_ID, MotorType.kBrushless);

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_NEO)
      .encoder.positionConversionFactor(ElevatorConstants.ELEVATOR_MOTOR_REDUCTION);

    motor1.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    posController = new PIDController(p, i, d);
    posController.setTolerance(ElevatorConstants.PID_ERROR_TOLERANCE);

    motor1.getEncoder().setPosition(0);
    motor2.getEncoder().setPosition(0);

    ElasticUtil.putDouble("Elevator P", () -> this.p, value -> { this.p = value; });
    ElasticUtil.putDouble("Elevator I", () -> this.i, value -> { this.i = value; });
    ElasticUtil.putDouble("Elevator D", () -> this.d, value -> { this.d = value; });
    ElasticUtil.putDouble("Elevator Position", this::getEncoder);
    ElasticUtil.putDouble("Elevator Pos 1", motor1.getEncoder()::getPosition);
    ElasticUtil.putDouble("Elevator Pos 2", motor2.getEncoder()::getPosition);
    ElasticUtil.putBoolean("Elevator At Setpoint", this::atSetpoint);

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
    motor2.set(-speed);
  }

  public boolean atSetpoint() {
    return posController.atSetpoint();
  }

  public double getEncoder() {
    return (motor1.getEncoder().getPosition() + motor2.getEncoder().getPosition()) / 2;
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
    posController.setPID(p, i, d);

    switch(currentState) {
      case IDLE:
      case BROKEN:
        break;
      case INTAKING:
      case HOME:
      case L1:
      case L2:
        if(!atSetpoint()) setSpeed(Utils.normalize(posController.calculate(getEncoder())));
        break;

      default:
        break;
    }

    // if(!checkSubsystem()) {
    //   setDesiredState(ElevatorStates.BROKEN);
    // }
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
        posController.setSetpoint(ElevatorConstants.INTAKING_POSITION);
      case HOME:
        posController.setSetpoint(ElevatorConstants.HOME_POSITION);
        break;
      case L1:
        posController.setSetpoint(ElevatorConstants.L1_POSITION);
        break;
      case L2:
        posController.setSetpoint(ElevatorConstants.L2_POSITION);
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
    INTAKING,
    HOME,
    L1,
    L2;
  }
}
