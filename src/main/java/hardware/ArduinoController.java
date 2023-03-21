package hardware;

import calc.Constants.LEDConstants;
import edu.wpi.first.wpilibj.I2C;

public class ArduinoController {
  
  public static final int ARDUINO_ADDRESS = 8;
  private int currentValue = -1;
  private boolean newLoop = true;

  //Sets up the Arduino over I2C on port 8
  public final I2C arduino = new I2C(I2C.Port.kOnboard, ARDUINO_ADDRESS);
  
  public void sendByte(int value) {
    
    if (value != currentValue && newLoop) {
      newLoop = false;
    
      currentValue = value;
          
      if (value == LEDConstants.BELLY_PAN_PURPLE || 
          value == LEDConstants.BELLY_PAN_YELLOW || 
          value == LEDConstants.BELLY_PAN_RAINBOW) 
      {
        arduino.writeBulk(new byte[] {((byte) (value)), ((byte) (value + 20))});
      }
      else {
        arduino.write(ARDUINO_ADDRESS, value);
      }
    }
  }

  public void reset() {
    this.newLoop = true;
  }
}
