/*package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingConstants;

//TODO: Clean this mess up

public class Lighting extends SubsystemBase {

    public enum State{PIT, SHOW, TELEOP, ENDGAME};
    State curLEDState = State.SHOW;

    private double backLeftBufferLen;
    private double backRightBufferLen;
    private double frontLeftBufferLen;
    private double frontRightBufferLen;
    private double turretBufferLen;

    private int m_rainbowFirstPixelHue;

    Timer LED_TIMER;

    int test = 0;

    AddressableLED backLeft_LEDStrip;
    AddressableLED backRight_LEDStrip;
    AddressableLED frontLeft_LEDStrip;
    AddressableLED frontRight_LEDStrip;
    AddressableLED turret_LEDStrip;

    AddressableLEDBuffer backLeft_LEDBuffer;
    AddressableLEDBuffer backRight_LEDBuffer;
    AddressableLEDBuffer frontLeft_LEDBuffer;
    AddressableLEDBuffer frontRight_LEDBuffer;
    AddressableLEDBuffer turret_LEDBuffer;

    DriverStation ds = DriverStation.getInstance();

    Random rand = new Random();



    public Lighting()
    {
        backLeft_LEDStrip = new AddressableLED(LightingConstants.backLeft_LEDPin);
        backRight_LEDStrip = new AddressableLED(LightingConstants.backRight_LEDPin);
        frontLeft_LEDStrip = new AddressableLED(LightingConstants.frontLeft_LEDPin);
        frontRight_LEDStrip = new AddressableLED(LightingConstants.frontRight_LEDPin);
        turret_LEDStrip = new AddressableLED(LightingConstants.turret_LEDPin);


        backLeft_LEDBuffer = new AddressableLEDBuffer(LightingConstants.LED_Count_BackLeft);
        backRight_LEDBuffer = new AddressableLEDBuffer(LightingConstants.LED_Count_BackRight);
        frontLeft_LEDBuffer = new AddressableLEDBuffer(LightingConstants.LED_Count_FrontLeft);
        frontRight_LEDBuffer = new AddressableLEDBuffer(LightingConstants.LED_Count_FrontRight);
        turret_LEDBuffer = new AddressableLEDBuffer(LightingConstants.LED_Count_Turret);


        backLeft_LEDStrip.setLength(backLeft_LEDBuffer.getLength());
        backRight_LEDStrip.setLength(backRight_LEDBuffer.getLength());
        frontLeft_LEDStrip.setLength(frontLeft_LEDBuffer.getLength());
        frontRight_LEDStrip.setLength(frontRight_LEDBuffer.getLength());
        turret_LEDStrip.setLength(turret_LEDBuffer.getLength());

        backLeft_LEDStrip.setData(backLeft_LEDBuffer);
        backRight_LEDStrip.setData(backRight_LEDBuffer);
        frontLeft_LEDStrip.setData(frontLeft_LEDBuffer);
        frontRight_LEDStrip.setData(frontRight_LEDBuffer);
        turret_LEDStrip.setData(turret_LEDBuffer);

        backLeft_LEDStrip.start();
        backRight_LEDStrip.start();
        frontLeft_LEDStrip.start();
        frontRight_LEDStrip.start();
        turret_LEDStrip.start();
 
        LED_TIMER = new Timer();
        LED_TIMER.start();
    }

    public void setLedState(State state)
    {
      curLEDState = state;
    }

    @Override
    public void periodic() {
      //update();


      //rainbow();

      /*for(int i = 0; i < 5; i++)
      {
        ballCount(i);
        pause(0.5);
      }

      for(int i = 5; i > 0; i--)
      {
        ballCount(i);
        pause(0.5);
      }*/

      /*if(test == 0)
      {
        betterSeq();
        test = 1;
      }*/
      
     /*if(ds.isOperatorControl())
      {
        setLedState(State.TELEOP);
      }
      else if(!ds.isAutonomous())
      {
        setLedState(State.SHOW);
      }

      if(curLEDState == State.SHOW)
      {
        //showAnimationSeq();
        clearStrip();
      }
      else if(curLEDState == State.TELEOP)
      {
        clearStrip();
      }
      else if(curLEDState == State.ENDGAME)
      {
        clearStrip();
      }
      
    }

    private void betterSeq()
    {
      quickFill(Color.kGold, 0.02);
      quickFill(Color.kBlue, 0.02);
      clearStrip();
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.3);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.3);
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.3);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.3);
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.3);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.3);
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.3);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.3);
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.3);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.1);
      clearStrip();
    }

    private void showAnimationSeq()
    {
      quickFill(Color.kGold, 0.02);
      quickFill(Color.kBlue, 0.02);
      quickFill(Color.kDarkTurquoise, 0.02);
      clearStrip();
      alternatingColors(Color.kGold, Color.kBlue);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.5);
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.5);
      alternatingColors(Color.kBlue, Color.kGold);
      pause(0.5);
      alternatingColors(Color.kGold, Color.kBlue);
      pause(0.5);
      alternatingColors(Color.kBlue, Color.kGold);
      clearStrip();
      fadeToColor(100);
      fadeFromColor(100);
      fadeToColor(50);
      fadeFromColor(50);
      middleFill(Color.kDeepPink);
      middleFill(Color.kFirstBlue);
      fillAtRandom();
      clearStrip();

    }

    private void pause(double time)
    {
      
    }

    private void clearStrip(AddressableLED led,AddressableLEDBuffer buffer)
    {
      for(int i = 0; i < buffer.getLength(); i++)
      {
        buffer.setRGB(i, 0, 0, 0);
      }
      led.setData(buffer);
    }
  
    private void fadeFromColor(int h, AddressableLED led, AddressableLEDBuffer buffer)
    {
      for(int i = 255; i > 0; i-=15)
      {
        for(int j = 0; j < buffer.getLength(); j++)
        {
          buffer.setHSV(j, h, 255, i);
        }
        led.setData(buffer);
      }
      clearStrip(led, buffer);
    }

    private void fadeToColor(int h, AddressableLED led, AddressableLEDBuffer buffer)
    {
      for(int i = 0; i < 255; i+=15)
      {
        for(int j = 0; j < buffer.getLength(); j++)
        {
          buffer.setHSV(j, h, 255, i);
        }
        led.setData(buffer);
        pause(0.1);
      }
    }


    private void rainbow(AddressableLED led, AddressableLEDBuffer buffer) {
      // For every pixel
      for (int i = 0; i < buffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
        // Set the value
        buffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;

      led.setData(buffer);
    }


    private void quickFill(Color c, double wait)
    {
      for(int i = 0; i < ledBuffer.getLength(); i++)
      {
        ledBuffer.setLED(i, c);
        led1.setData(ledBuffer);
        Timer.delay(wait);
      }
    }






    private void alternatingColors(Color c1, Color c2)
    {
        for(int i = 0; i < ledBuffer.getLength() - 1; i++)
        {
          if(i % 2 == 0)
          {
            ledBuffer.setLED(i, c1);
          }
          else
          {
            ledBuffer.setLED(i, c2);
          }    
        }
        led1.setData(ledBuffer);

    }

    private void fillAtRandom()
    {
        int[] used = new int[ledBuffer.getLength()];
        int lights = 0;
      
        for(int i = 0; i < ledBuffer.getLength() - 1; i++)
        {
          used[i] = 0;
        }
      
        while(lights < ledBuffer.getLength() - 1)
        {
          int j = rand.nextInt(ledBuffer.getLength() - 1);
          if(used[j] != 1)
          {
            ledBuffer.setHSV(j, rand.nextInt(180), 255, 128);
            used[j] = 1;
            lights++;
            led1.setData(ledBuffer);
            Timer.delay(0.05);
          }
        }
    }

    private void middleFill(Color c)
    {

        for(int i = 0; i < ((ledBuffer.getLength() - 1) / 2); i++)
        {
            ledBuffer.setLED(ledBuffer.getLength()/2 + i, c);
            ledBuffer.setLED(ledBuffer.getLength()/2 - i, c);
            led1.setData(ledBuffer);
            Timer.delay(0.05);
        }
        for(int i = 0; i < (ledBuffer.getLength() / 2); i++)
        {
            ledBuffer.setRGB(i, 0,0,0);
            ledBuffer.setRGB((ledBuffer.getLength() -1) - i, 0,0,0);
            led1.setData(ledBuffer);
            Timer.delay(0.05);
        }

    }




    private void ballCount(int balls)
    {
      double a = Math.ceil(ledBufferLen / 6);
      int b = (int)a;
      int groups[][] = new int[5][b];
      int n = 0;
      
      for(int i = 0; i < ledBuffer.getLength(); i++)
      {
        //leds[i*(NUM_LEDS / 5) - 1] = CRGB::Blue;
        if(i%(ledBuffer.getLength() / 5) ==0)
        {
          ledBuffer.setLED(i, Color.kDarkRed);
        }
        else
        {
          groups[n / 5][n % 5] = i;
          n++;
        }
      }


      switch(balls)
      {
        case 0:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kBlack);
          ledBuffer.setLED(groups[1][j], Color.kBlack);
          ledBuffer.setLED(groups[2][j], Color.kBlack);
          ledBuffer.setLED(groups[3][j], Color.kBlack);
          ledBuffer.setLED(groups[4][j], Color.kBlack);
          }
          break;
        case 1:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[1][j], Color.kBlack);
          ledBuffer.setLED(groups[2][j], Color.kBlack);
          ledBuffer.setLED(groups[3][j], Color.kBlack);
          ledBuffer.setLED(groups[4][j], Color.kBlack);
          }
          break;
        case 2:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[1][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[2][j], Color.kBlack);
          ledBuffer.setLED(groups[3][j], Color.kBlack);
          ledBuffer.setLED(groups[4][j], Color.kBlack);
          }
          break;
        case 3:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[1][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[2][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[3][j], Color.kBlack);
          ledBuffer.setLED(groups[4][j], Color.kBlack);
          }
          break;
        case 4:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[1][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[2][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[3][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[4][j], Color.kBlack);
          }
          break;
        case 5:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[1][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[2][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[3][j], Color.kDeepSkyBlue);
          ledBuffer.setLED(groups[4][j], Color.kDeepSkyBlue);
          }
          break;
        default:
          for(int j = 0; j < 5; j++ )
          {
          ledBuffer.setLED(groups[0][j], Color.kBlack);
          ledBuffer.setLED(groups[1][j], Color.kBlack);
          ledBuffer.setLED(groups[2][j], Color.kBlack);
          ledBuffer.setLED(groups[3][j], Color.kBlack);
          ledBuffer.setLED(groups[4][j], Color.kBlack);
          }
          break;


      }
      
      /*for (int i = 0; i <balls; i++)
      {
       for(int j = 0; j < 5; j++ )
       {
        ledBuffer.setLED(groups[i][j], Color.kDeepSkyBlue);
       }
      }*/
 /*     led1.setData(ledBuffer);
    }

    private void shooterSpeed(int curSpeed, int targetSpeed)
    {
      // Start on sides and fill to center based on % of targetSpeed once target is hit flash


      
      /*for(int i = 0; i < (ledBuffer.getLength() / 2); i++)
      {
        ledBuffer.setHSV(i, 100, 255, 100);
        ledBuffer.setHSV(ledBuffer.getLength() - 1, 100, 255, 100)
        delay(wait);
      }*/



 /*   }


    public void update()
    {
      SmartDashboard.putString("ledState", curLEDState.toString());
      SmartDashboard.putBoolean("dsEnabled", ds.isEnabled());
      SmartDashboard.putBoolean("dsTeleOP", ds.isOperatorControl());
    }


}*/