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
import frc.robot.handlers.CheckableSubsystem;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

// This is for the algae
public class S_Pivot extends SubsystemBase implements CheckableSubsystem {
  private boolean initialized = false;
  private boolean status = false;

  private SparkMax motor;

  private static S_Pivot m_Instance;

  private PIDController angleController;
  private double p = 0.03, i = 0, d = 0;

  public S_Pivot() {
    motor = new SparkMax(CANConstants.PIVOT_ID, MotorType.kBrushless);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_NEO);

    angleController = new PIDController(p, i, d);
    angleController.setTolerance(PivotConstants.PID_ERROR_TOLERANCE);

    motor.getEncoder().setPosition(0);

    ElasticUtil.putDouble("Pivot P", () -> this.p, value -> { this.p = value; });
    ElasticUtil.putDouble("Pivot I", () -> this.i, value -> { this.i = value; });
    ElasticUtil.putDouble("Pivot D", () -> this.d, value -> { this.d = value; });
    ElasticUtil.putDouble("Pivot Position", motor.getEncoder()::getPosition);
    ElasticUtil.putBoolean("Pivot At Setpoint", this::atSetpoint);

    initialized = true;
  }

  public static S_Pivot getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_Pivot();
    }

    return m_Instance;
  }

  public void set(double speed) {
    motor.set(Utils.normalize(speed));
  }

  public void setSetpoint(double setpoint) {
    angleController.setSetpoint(setpoint);
  }

  public void moveToSetpoint() {
    set(angleController.calculate(motor.getEncoder().getPosition()));
  }

  public boolean atSetpoint() {
    return angleController.atSetpoint();
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
}
