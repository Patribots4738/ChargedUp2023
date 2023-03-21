package hardware;

import calc.Constants.LEDConstants;
import edu.wpi.first.wpilibj.I2C;
import java.util.Queue;

public class ArduinoController {

  //Sets up the Arduino over I2C on port 8
  private final I2C arduino = new I2C(I2C.Port.kOnboard, LEDConstants.ARDUINO_ADDRESS);
  private Queue<Integer> queue;
  private int currentBellyPanState = -1;
  private int currentArmState = -1;


  public void periodic() {
    // Write the latest byte in the queue to the arduino
    // If it exists
    if (queue.peek() != null) {
      if (queue.peek() >= 20) {
        // If the state is the same as the current state, don't send it
        // and remove it from the queue
        if (currentArmState == queue.peek()) {
          queue.remove();
          return;
        }
        // Set the current state to our value
        currentArmState = queue.peek();
      }
      else {
        // If the state is the same as the current state, don't send it
        // and remove it from the queue
        if (currentBellyPanState == queue.peek()) {
          queue.remove();
          return;
        }
        // Set the current state to our value
        currentBellyPanState = queue.peek();
      }
      // Send the latest byte in the queue to the arduino
      sendByte();
    } 
  }

  public void setLEDState(int state) {
    // Add the state to the queue
    queue.add(state);
  }

  public void sendByte() {
    arduino.write(LEDConstants.ARDUINO_ADDRESS, queue.poll());
  }
}
