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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SpitterConstants;

// This is for the coral
public class Spitter extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor1, motor2;
  private final DigitalInput proximitySensor;

  private static Spitter m_instance;

  private SpitterStates desiredState = SpitterStates.IDLE, currentState = SpitterStates.IDLE;

  /** Creates a new Rollers. */
  public Spitter() {
    motor1 = new SparkMax(CANConstants.ROLLER_MOTOR_ONE_ID, MotorType.kBrushless);
    motor2 = new SparkMax(CANConstants.ROLLER_MOTOR_TWO_ID, MotorType.kBrushless);

    SparkMaxConfig spitterConfig = new SparkMaxConfig();

    spitterConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.CURRENT_LIMIT_NEO);

    motor1.configure(spitterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(spitterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    proximitySensor = new DigitalInput(CANConstants.PROXIMITY_SENSOR_CHANNEL);

    initialized = true;
  }
  
  public static Spitter getInstance() {
    if(m_instance == null) {
      m_instance = new Spitter();
    }

    return m_instance;
  }

  public void runSpitter() {
    motor1.set(SpitterConstants.SPITTER_SPEED);
    motor2.set(-SpitterConstants.SPITTER_SPEED);
  }

  @Override
  public void stop() {
    motor1.stopMotor();
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

  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
      case BROKEN:
        break;
      case INTAKING:
        if(!proximitySensor.get()) runSpitter();
      case RUNNING:
        runSpitter();
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(SpitterStates.BROKEN);
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
      case RUNNING:
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  public void setDesiredState(SpitterStates state) {
    if(desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  public SpitterStates getState() {
    return currentState;
  }

  public enum SpitterStates {
    IDLE,
    BROKEN,
    INTAKING,
    RUNNING;
  }
}
