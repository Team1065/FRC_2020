package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SerialComms extends SubsystemBase {

    private SerialPort arduino;


    /*
    *Commands:
    *show    -- Flashy showoff mode
    *auto    -- Light feedback during auto
    *teleop  -- Light feedback during teleop 
        *setColor -- TODO
        *flashColor -- TODO
        ?more commands during teleop?
    ?endgame -- Light feedback during endgame or just change to show mode
    */


    public SerialComms()
    {
        try 
        {
            arduino = new SerialPort(9600, SerialPort.Port.kOnboard);
        }
        catch(Exception e)
        {
            System.out.println(e.getStackTrace());
        }
    }

    public void sendData(String data)
    {
        arduino.writeString(data);
        System.out.println("Sent:" + data);
    }

    @Override
    public void periodic() 
    {
        if(arduino.getBytesReceived() > 0)
        {
            System.out.print(arduino.readString());
        }
    }


}