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
import frc.robot.handlers.CheckableSubsystem;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

public class S_Elevator extends SubsystemBase implements CheckableSubsystem {
  private boolean initialized = false;
  private boolean status = false;

  private SparkMax motor1, motor2;

  private static S_Elevator m_Instance;

  private PIDController posController;
  private double p = 2, i = 0, d = 0;

  public S_Elevator() {
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
    ElasticUtil.putDouble("Elevator Position", this::getEncoderPosition);
    ElasticUtil.putDouble("Elevator Pos 1", motor1.getEncoder()::getPosition);
    ElasticUtil.putDouble("Elevator Pos 2", motor2.getEncoder()::getPosition);
    ElasticUtil.putBoolean("Elevator At Setpoint", this::atSetpoint);

    initialized = true;
  }

  public static S_Elevator getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_Elevator();
    }

    return m_Instance;
  }

  public void setSpeed(double speed) {
    speed = Utils.normalize(speed);
    motor1.set(-speed);
    motor2.set(speed);
  }

  public void setSetpoint(double setpoint) {
    posController.setSetpoint(setpoint);
  }

  public void moveToSetpoint() {
    setSpeed(posController.calculate(getEncoderPosition()));
  }

  public boolean atSetpoint() {
    return posController.atSetpoint();
  }

  public double getEncoderPosition() {
    return (motor2.getEncoder().getPosition() - motor1.getEncoder().getPosition()) / 2;
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
    status = initialized;

    return status;
  }
}
