package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import math.*;

public class Arm {

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