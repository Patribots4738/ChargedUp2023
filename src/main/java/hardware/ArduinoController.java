package hardware;

import calc.Constants.LEDConstants;
import edu.wpi.first.wpilibj.I2C;

import java.util.LinkedList;
import java.util.Queue;

public class ArduinoController {

  //Sets up the Arduino over I2C on port 8
  private final I2C arduino = new I2C(I2C.Port.kOnboard, LEDConstants.ARDUINO_ADDRESS);
  private Queue<Integer> queue = new LinkedList<Integer>();
  private int currentBellyPanState = -1;
  private int currentArmState = -1;
  private boolean everyOtherLoop = false;

  public void periodic() {
      // Toggle our boolean to not overload the arduino with requests
      everyOtherLoop = !everyOtherLoop;
      // Write the latest byte in the queue to the arduino
      // If it exists
      if (queue.peek() != null) {
          int nextRequest = queue.peek();
          // All arm requests are within 20 and 30... for now.
          if (nextRequest >= 20 && nextRequest <= 30) {
              // If the state is the same as the current state, don't send it
              // and remove it from the queue
            //   if (currentArmState == nextRequest) {
            //       queue.remove();
            //       return;
            //   }
              // Set the current state to our value
              currentArmState = nextRequest;
          } else {
              // If the state is the same as the current state, don't send it
              // and remove it from the queue
            //   if (currentBellyPanState == nextRequest) {
            //       queue.remove();
            //       return;
            //   }
              // Set the current state to our value
              currentBellyPanState = nextRequest;
          }
          // Send the latest byte in the queue to the arduino
          sendByte();
      }
  }

  public void setLEDState(int state) {
      // Add the state to the queue
      // If the state is the same as the current state, don't send it
      if (!queue.contains(state) &&
              state != currentArmState &&
              state != currentBellyPanState && 
              state+100 != currentBellyPanState)
      {
        queue.offer(state);
      }

  }

  public void setLEDState(int state, boolean override) {
    // Add the state to the queue
    if (override) {
        queue.offer(state);
    }
    else {
        setLEDState(state);
    }
  }

  public void sendByte() {
    // Only send bytes every other loop to not overload arduino
    if (everyOtherLoop) {
        // Send the latest queue value to the arduino,
        // Then, remove the latest value from the queue
        if (queue.peek() != null) {
            // System.out.println("Sending byte: " + queue.peek());
            arduino.write(LEDConstants.ARDUINO_ADDRESS, queue.poll());
        }    
    }
  }

  public void setBellyPanState(int state) {
    this.currentBellyPanState = state;
  }

  public void setArmState(int state) {
    this.currentArmState = state;
  }
}
