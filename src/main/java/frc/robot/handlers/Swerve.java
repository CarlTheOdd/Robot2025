// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.OI;
import frc.robot.subsystems.S_Swerve;

public class Swerve extends SubsystemBase implements StateSubsystem {
  private SwerveStates desiredState = SwerveStates.IDLE, currentState = SwerveStates.IDLE;
  private S_Swerve swerve = S_Swerve.getInstance();
  private static Swerve m_Instance;

  private Swerve() {}

  public static Swerve getInstance() {
    if(m_Instance == null) {
      m_Instance = new Swerve();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
  }

  @Override
  public void update() {
    // if((int) NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tid").getInteger(-1) != -1 && !trackingTag) {
    //   OI.rumbleControllers();
    //   trackingTag = true;
    // }

    switch(currentState) {
      case IDLE:
        setDesiredState(SwerveStates.DRIVE);
      case BROKEN:
        break;
      case DRIVE:
        if(DriverStation.isTeleopEnabled()) {
          swerve.drive(
              MathUtil.applyDeadband(OI.driverController.getLeftY(), SwerveConstants.DRIVING_DEADBAND),
              MathUtil.applyDeadband(OI.driverController.getLeftX(), SwerveConstants.DRIVING_DEADBAND),
              MathUtil.applyDeadband(OI.driverController.getRightX(), SwerveConstants.DRIVING_DEADBAND),
              true, SwerveConstants.SPEED_SCALE);
        }
        break;
      case AIMING:
        // double targetOffset = NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("ty").getDouble(0);
        // double totalOffset = (LimelightConstants.LIMELIGHT_ANGLE + targetOffset) * (3.14159 / 180);

        // double distanceToTag = (currTargetHeight - LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(totalOffset);

        // double xOffset = distanceToTag * Math.sin(NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tx").getDouble(0));
        // double yOffset = distanceToTag * Math.cos(NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tx").getDouble(0));

        // drive(xOffset, yOffset);
        switch((int) NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tid").getInteger(-1)) {
          case 0:
            swerve.angleController.setSetpoint(0);
            break;
          case 1:
            swerve.angleController.setSetpoint(45);
            break;
          case 2:
            swerve.angleController.setSetpoint(90);
            break;
          case 3:
            swerve.angleController.setSetpoint(135);
            break;
          case 4:
            swerve.angleController.setSetpoint(180);
            break;
          case 5:
            swerve.angleController.setSetpoint(225);
            break;
          case 6:
            swerve.angleController.setSetpoint(270);
            break;
          case 7:
            swerve.angleController.setSetpoint(315);
            break;
        }
        swerve.drive(
          OI.driverController.getLeftY(),
          OI.driverController.getLeftX(),
          -swerve.angleController.calculate(swerve.getHeading()),
          true, SwerveConstants.SPEED_SCALE);
        break;
      case LOCKED:
        swerve.setX();
        break;

      default:
        break;
    }

    // if(!checkSubsystem()) {
    //   setDesiredState(SwerveStates.BROKEN);
    // }
  }

  /**
   * Handles moving from one state to another
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
      case BROKEN:
        swerve.stop();
        break;
      case DRIVE:
        break;
      case AIMING:
        // switch((int) NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tid").getInteger(-1)) {
        //   // Reef IDs
        //   case 6:
        //   case 19:
        //     targetHeading = 45;
        //     currTarget = Targets.REEF;
        //     break;
        //   case 7:
        //   case 18:
        //     targetHeading = 0;
        //     currTarget = Targets.REEF;
        //     break;
        //   case 8:
        //   case 17:
        //     targetHeading = 315;
        //     currTarget = Targets.REEF;
        //     break;
        //   case 9:
        //   case 22:
        //     targetHeading = 225;
        //     currTarget = Targets.REEF;
        //     break;
        //   case 10:
        //   case 21:
        //     targetHeading = 180;
        //     currTarget = Targets.REEF;
        //     break;
        //   case 11:
        //   case 20:
        //     targetHeading = 135;
        //     currTarget = Targets.REEF;
        //     break;

        //   // Processor IDs
        //   case 3:
        //   case 16:
        //     targetHeading = 90;
        //     currTarget = Targets.PROCESSOR;
        //     break;

        //   // Source IDs
        //   case 1:
        //   case 13:
        //     targetHeading = 225;
        //     currTarget = Targets.SOURCE;
        //     break;
        //   case 2:
        //   case 12:
        //     targetHeading = 135;
        //     currTarget = Targets.SOURCE;
        //     break;

        //   default:
        //     break;
        // }

        // switch(currTarget) {
        //   case REEF:
        //     currTargetHeight = LimelightConstants.REEF_TAG_HEIGHT;
        //     break;
        //   case PROCESSOR:
        //     currTargetHeight = LimelightConstants.PROCESSOR_TAG_HEIGHT;
        //     break;
        //   case SOURCE:
        //     currTargetHeight = LimelightConstants.SOURCE_TAG_HEIGHT;
        //     break;
          
        //   default:
        //     // This should never run
        //     currTargetHeight = 0;
        //     break;
        // }
        break;
      case LOCKED:
        swerve.setX();
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  /**
   * Sets the desired state of the subsystem
   * @param state Desired state
   */
  @Override
  public void setDesiredState(State state) {
    if(this.desiredState != state) {
      desiredState = (SwerveStates) state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public SwerveStates getState() {
    return currentState;
  }

  public Trigger bindState(Trigger button, SwerveStates onTrue, SwerveStates onFalse) {
    return button
      .onTrue(new InstantCommand(() -> setDesiredState(onTrue), m_Instance))
      .onFalse(new InstantCommand(() -> setDesiredState(onFalse), m_Instance));
  }

  /**
   * The list of possible states for this subsystem
   */
  public enum SwerveStates implements State {
    IDLE,
    BROKEN,
    /** Regular control of the robot */
    DRIVE,
    /** Uses the angle PID controller minimize the offset provided by aimingAngle.
     * This takes away yaw control from the driver.
     */
    AIMING,
    /** Removes all control and locks the wheels in an X formation */
    LOCKED;
  }

  // public enum Targets {
  //   REEF,
  //   PROCESSOR,
  //   SOURCE;
  // }
}
