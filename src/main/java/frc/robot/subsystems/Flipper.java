// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Flipper extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean initialized, status = false;

  private SparkMax motor;
  
  public Flipper() {
    motor = new SparkMax(CANConstants.FLIPPER_ID, MotorType.kBrushless);

    initialized = true;
  }

  @Override
  public void periodic() {}

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

  }

  @Override
  public void handleStateTransition() {

  }

  public void setDesiredState() {

  }

  public enum FlipperStates {
    IDLE,
    BROKEN;
  }
}
