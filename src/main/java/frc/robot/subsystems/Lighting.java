package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingConstants;

//TODO: Clean this mess up

public class Lighting extends SubsystemBase {

    public enum State{PIT, SHOW, TELEOP};
    State curLEDState = State.SHOW;

    DriverStation ds = DriverStation.getInstance();
    SerialPort arduino;




    public Lighting()
    {
        try
        {
          arduino = new SerialPort(9600, Port.kUSB1);
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

    @Override
    public void periodic() {
     
      if(ds.isOperatorControl() && ds.isFMSAttached() && ds.isEnabled())
      {
        setLedState(State.TELEOP);
      }
      else if(ds.isOperatorControl() && !ds.isFMSAttached() && ds.isEnabled())
      {
        setLedState(State.PIT);
      }
      else
      {
        setLedState(State.SHOW);
      }

      if(curLEDState == State.SHOW)
      {
        // send mode
      }
      else if(curLEDState == State.TELEOP)
      {
        // send mode
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
        arduino.writeString("m0");
      }
      else if(s == State.SHOW)
      {
        arduino.writeString("m1");
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