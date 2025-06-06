package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.S_Spitter;

public class Spitter extends SubsystemBase implements StateSubsystem {
  private SpitterStates desiredState, currentState = SpitterStates.IDLE;
  private S_Spitter spitter = S_Spitter.getInstance();
  private static Spitter m_Instance;

  private Spitter() {}

  public static Spitter getInstance() {
    if(m_Instance == null) {
      m_Instance = new Spitter();
    }

    return m_Instance;
  }

  @Override
  public void setDesiredState(State state) {
    if(desiredState != state) {
      desiredState = (SpitterStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        spitter.stop();
        break;
      case BROKEN:
        spitter.stop();
        break;
      case RUNNING:
        spitter.runSpitter();
        break;

      default:
        break;
    }
  }

  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case RUNNING:
        break;
      
      default:
        break;
    }

    if(!spitter.checkSubsystem()) {
      setDesiredState(SpitterStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    update();
  }

  public SpitterStates getState() {
    return currentState;
  }

  public enum SpitterStates implements State {
    IDLE,
    BROKEN,
    RUNNING;
  }
}
