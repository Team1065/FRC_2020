package frc.robot.subsystems;

import java.security.CryptoPrimitive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {

    public enum State{PIT, SHOW, TELEOP};
    State curLEDState = State.SHOW;
    State oldState = curLEDState;

    DriverStation ds = DriverStation.getInstance();
    SerialPort arduino;

    /*  Commands:
        Modes:
          m0 - Show
          m1 - Pit
          m2 - Teleop
          m3 - Endgame
        Balls:
          b0 - 0 balls
          b1 - 1 balls
          b2 - 2 balls
          b3 - 3 balls
          b4 - 4 balls
          b5 - 5 balls
        Shooter:
          t1 - Ready to shoot
          t0 - Not ready to shoot
        Climber (endgame):
          c1 - Fully down
          c0 - Not fully down
        

    */


    public Lighting()
    {
        try
        {
          arduino = new SerialPort(9600, Port.kUSB1);
          arduino.writeString("Hello Arduino!");
        }
        catch(Exception e)
        {
          System.out.println("Failed to connect to arduino");
        }
    }

    public void setLedState(State state)
    {
      curLEDState = state;
    }

    public boolean changedState()
    {
      if(curLEDState != oldState)
      {
        oldState = curLEDState;
        return true;
      }
      else
      {
        return false;
      }
    }

    @Override
    public void periodic() {
     
      if(ds.isOperatorControl() && ds.isFMSAttached() && ds.isEnabled())
      {
        setLedState(State.TELEOP);
      }
      else if(!ds.isFMSAttached() && ds.isEnabled())
      {
        setLedState(State.PIT);
      }
      else if(!ds.isEnabled())
      {
        setLedState(State.SHOW);
      }

      if(curLEDState == State.SHOW)
      {
        if(changedState())
        {
          sendState(State.SHOW);
        }
      }
      else if(curLEDState == State.TELEOP)
      {
        if(changedState())
        {
          sendState(State.TELEOP);
        }
        
      }
      if(curLEDState == State.PIT)
      {
        if(changedState())
        {
          sendState(State.PIT);
        }
        
      }
    }

    private void sendBallCount(int count)
    {
      arduino.writeString("b"+count);
    }
    
    private void sendTurretStatus(boolean uptospeed)
    {
      if(uptospeed)
      {
        arduino.writeString("t1");
      }
      else
      {
        arduino.writeString("t0");
      }
    }

    private void sendState(State s)
    {
      if(s == State.PIT)
      {
        arduino.writeString("m1");
      }
      else if(s == State.SHOW)
      {
        arduino.writeString("m0");
      }
      else if(s == State.TELEOP)
      {
        arduino.writeString("m2");
      }
    }


    public void update()
    {
      SmartDashboard.putString("ledState", curLEDState.toString());
    }
}