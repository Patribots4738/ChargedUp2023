package hardware;

import edu.wpi.first.wpilibj.I2C;

public class ArduinoController {
  
  public static final int ARDUINO_ADDRESS = 8;
  public final I2C arduino = new I2C(I2C.Port.kOnboard, ARDUINO_ADDRESS); //Sets up the Arduino over I2C on port 8
  private int currentValue = 0;
  public void sendByte(int value) {
    if (value != currentValue) {
      currentValue = value;
      arduino.write(ARDUINO_ADDRESS, value);
    }
  }

}
