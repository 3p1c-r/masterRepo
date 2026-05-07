package frc.robot.simplesubsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleJointedArm extends SubsystemBase {
    // ---------------------------------------------------------------------------
    // Hardware Variables
    // ---------------------------------------------------------------------------
    private final SparkMax m_motor;
    private final DigitalInput m_forwardLimitSwitch;
    private final DigitalInput m_backwardLimitSwitch;
    private final double m_constArmSpeed;

    // ---------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------
    public SingleJointedArm(){
        m_motor = new SparkMax(20, SparkLowLevel.MotorType.kBrushless);
        m_forwardLimitSwitch = new DigitalInput(12);
        m_backwardLimitSwitch = new DigitalInput(13);
        m_constArmSpeed = 0.35;
    }

    public void stopMoving(){m_motor.set(0);}

    public void moveManual(double speed){
        SmartDashboard.putNumber("Arm Speed", speed);
//        SmartDashboard.putNumber("Arm Position",      getPosition());
//        SmartDashboard.putNumber("Arm Target",        m_targetPosition);
//        SmartDashboard.putBoolean("Arm At Target",    isAtTarget());
//        SmartDashboard.putBoolean("Arm Bottom Limit", isAtBottom());
//        SmartDashboard.putBoolean("Arm Top Limit",    isAtTop());
//        SmartDashboard.putBoolean("ArmPIDModeOn?", isPidMode());

        m_motor.set(speed);
//        m_targetPosition = getPosition();

        if((m_forwardLimitSwitch.get() && speed < 0) || (m_backwardLimitSwitch.get() && speed > 0)){stopMoving();return;}
//        if (isAtBottom()) {resetEncoder();}
    }

    public Command armForward(){
        return startEnd(
                () -> { //extend the intake
                    m_motor.set(m_constArmSpeed);
                    System.out.println("Arm Forward");
                    SmartDashboard.putBoolean("armForward?", true);
                    SmartDashboard.putBoolean("armMoving?", true);

                    if(m_forwardLimitSwitch.get() && m_motor.get() > 0){stopMoving();}
                },
                () -> { //stop extending the intake
                    stopMoving();
                    System.out.println("Arm Stopped");
                    SmartDashboard.putBoolean("armForward?", false);
                    SmartDashboard.putBoolean("armMoving?", false);
                }
        );
    }

    public Command armBackward(){
        return startEnd(
                () -> { //extend the intake
                    m_motor.set(m_constArmSpeed);
                    System.out.println("Arm Backward");
                    SmartDashboard.putBoolean("armBackward?", true);
                    SmartDashboard.putBoolean("armMoving?", true);

                    if(m_backwardLimitSwitch.get() && m_motor.get() < 0){stopMoving();}
                },
                () -> { //stop extending the intake
                    stopMoving();
                    System.out.println("Arm Stopped");
                    SmartDashboard.putBoolean("armBackward?", false);
                    SmartDashboard.putBoolean("armMoving?", false);
                }
        );
    }
}
