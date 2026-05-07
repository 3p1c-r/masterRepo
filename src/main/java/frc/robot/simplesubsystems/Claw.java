package frc.robot.simplesubsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    // ---------------------------------------------------------------------------
    // Hardware Variables
    // ---------------------------------------------------------------------------
    private final PWMSparkMax m_motor;
    private final DigitalInput m_outerLimitSwitch;
    private final DigitalInput m_innerLimitSwitch;
    private final double m_constClawSpeed;

    // ---------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------
    public Claw(){
        m_motor = new PWMSparkMax(20);
        m_outerLimitSwitch = new DigitalInput(12);
        m_innerLimitSwitch = new DigitalInput(13);
        m_constClawSpeed = 0.25;
    }

    public void stopMoving(){m_motor.set(0);}

    public Command openClaw(){
        return startEnd(
                () -> { //extend the intake
                    m_motor.set(m_constClawSpeed);
                    System.out.println("Claw Opening");
                    SmartDashboard.putBoolean("clawOpening?", true);
                    SmartDashboard.putBoolean("clawMoving?", true);

                    if(m_outerLimitSwitch.get() && m_motor.get() > 0){stopMoving();}
                },
                () -> { //stop extending the intake
                    stopMoving();
                    System.out.println("Claw Stopped");
                    SmartDashboard.putBoolean("clawOpening?", false);
                    SmartDashboard.putBoolean("clawMoving?", false);
                }
        );
    }

    public Command closeClaw(){
        return startEnd(
                () -> { //extend the intake
                    m_motor.set(m_constClawSpeed);
                    System.out.println("Claw Closing");
                    SmartDashboard.putBoolean("clawClosing?", true);
                    SmartDashboard.putBoolean("clawMoving?", true);

                    if(m_innerLimitSwitch.get() && m_motor.get() < 0){stopMoving();}
                },
                () -> { //stop extending the intake
                    stopMoving();
                    System.out.println("Claw Stopped");
                    SmartDashboard.putBoolean("clawClosing?", false);
                    SmartDashboard.putBoolean("clawMoving?", false);
                }
        );
    }

}
