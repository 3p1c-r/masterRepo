package frc.robot.simplesubsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public interface complexSubsystemInterface {
    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static void stop(SparkMax motor){
        motor.set(0);
    }

    public static void stop(PWMSparkMax motor){
        motor.set(0);
    }

    public default boolean isAtLimit(DigitalInput limitSwitch){return limitSwitch.get();}

    public default void resetEncoder(Encoder encoder) {encoder.reset();}




}
