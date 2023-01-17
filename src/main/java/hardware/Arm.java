package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import math.*;

public class Arm {

  /**
   * What the arm positions look like and the index in the array
   *         4
   * O        __     8
   *  1      |      7
   *       3 | 5
   *    2  |||||  6
   */

  int[][] armPos = {{0, 2}, {1, 1}, {2, 0}, {3, 1}, {4, 4}, {5, 1}, {6, 0}, {7, 2}, {8, 3}};
  int armPosIndex = 0;
  
  ArmCalcuations ArmCalculations = new ArmCalcuations();

  CANSparkMax lowerArm = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax upperArm = new CANSparkMax(10, MotorType.kBrushless);

  public void drive(double armX, double armY) {

      double Q2 = ArmCalculations.getQ2(armX, armY);
      double Q1 = ArmCalculations.getQ1(armX, armY, Q2);


      // lowerArm.set();
      // upperArm.set();
  }
    
}