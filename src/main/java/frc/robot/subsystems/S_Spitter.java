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
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SpitterConstants;
import frc.robot.handlers.CheckableSubsystem;

// This is for de-algaefying
public class S_Spitter extends SubsystemBase implements CheckableSubsystem {
  private boolean initialized = false;
  private boolean status = false;

  private SparkMax motor;

  private static S_Spitter m_Instance;

  public S_Spitter() {
    motor = new SparkMax(CANConstants.SPITTER_ID, MotorType.kBrushless);

    SparkMaxConfig spitterConfig = new SparkMaxConfig();

    spitterConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_NEO);

    motor.configure(spitterConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    initialized = true;
  }
  
  public static S_Spitter getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_Spitter();
    }

    return m_Instance;
  }

  public void runSpitter() {
    motor.set(SpitterConstants.SPITTER_SPEED);
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
}
