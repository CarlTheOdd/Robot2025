// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.handlers.CheckableSubsystem;
import frc.utils.Utils;

public class S_AlgaeIntake extends SubsystemBase implements CheckableSubsystem {
  private boolean initialized = false;
  private boolean status = false;

  private SparkMax motor;
  
  private static S_AlgaeIntake m_Instance;

  private S_AlgaeIntake() {
    motor = new SparkMax(CANConstants.ALGAE_INTAKE_ID, MotorType.kBrushless);

    SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();

    algaeIntakeConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_NEO);

    motor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    initialized = true;
  }

  public static S_AlgaeIntake getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_AlgaeIntake();
    }

    return m_Instance;
  }

  public void set(double speed) {
    motor.set(Utils.normalize(speed));
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
