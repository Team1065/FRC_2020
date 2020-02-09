package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {

    public enum State{SHOW, TELEOP, ENDGAME};
    State curLEDState;

    public int LED_COUTN1 = 300;
    private int m_rainbowFirstPixelHue;

    AddressableLED led1;
    AddressableLEDBuffer ledBuffer;



    public Lighting()
    {
        led1 = new AddressableLED(9);

        ledBuffer = new AddressableLEDBuffer(LED_COUTN1);
        led1.setLength(ledBuffer.getLength());

        led1.setData(ledBuffer);
        led1.start();
    }

    @Override
    public void periodic() {
      // Fill the buffer with a rainbow
      rainbow();
      // Set the LEDs
      led1.setData(ledBuffer);
    }
  
    private void rainbow() {
      // For every pixel
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
        // Set the value
        ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    }

}