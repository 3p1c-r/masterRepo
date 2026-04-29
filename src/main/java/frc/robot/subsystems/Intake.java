package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SubsystemConstants;

import static frc.robot.Constants.SubsystemConstants.*;
/* An intake subsystem that can:
* Extend outwards
* Retract inwards
* Take in game pieces
* Spit out game pieces (if they're stuck)
*
* The intake will stop extending or retracting if it hits one of the limit switches.
* */

public class Intake extends SubsystemBase{

    //Declare motors, speeds, default intake extension direction, and limit switches
    private final PWMSparkMax intakeMotor;
    private final PWMSparkMax extendMotor;

    private final double intakeSpeed;
    private final double extendSpeed;

    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    //Initialize the intake

    public Intake(){
        this.intakeMotor = new PWMSparkMax(intakeMotorPort); //Constants File
        this.extendMotor = new PWMSparkMax(extendIntakeMotorPort); //Constants File

        this.intakeSpeed = ConstIntakeSpeed; //Constants File
        this.extendSpeed = ConstExtendIntakeSpeed; //Constants File
    }

    //Stopper Methods
    public void stopIntake(){intakeMotor.set(0);}
    public void stopExtension(){extendMotor.set(0);}

    //Lambda Commands
    public Command runIntake2(){
        return startEnd(
                () -> { //run intake
                    intakeMotor.set(intakeSpeed);
                    System.out.println("Intake Command Called");
                    SmartDashboard.putBoolean("Intaking?", true);
                    SmartDashboard.putBoolean("IntakeRunning?", true);
                },
                () -> { //stop intake3
                    stopIntake();
                    System.out.println("Intake Command Stopped");
                    SmartDashboard.putBoolean("Intaking?", false);
                    SmartDashboard.putBoolean("IntakeRunning?", false);
                }
        );
    }

    public Command runOuttake2(){
        return startEnd(
                () -> { //outtake
                    intakeMotor.set(-intakeSpeed);
                    System.out.println("Outtake Command Called");
                    SmartDashboard.putBoolean("Outtaking?", true);
                    SmartDashboard.putBoolean("IntakeRunning?", true);
                },
                () -> { //stop outtake
                    stopIntake();
                    System.out.println("Intake Command Stopped");
                    SmartDashboard.putBoolean("Outtaking?", false);
                    SmartDashboard.putBoolean("IntakeRunning?", false);
                }
        );
    }

    public Command extendIntake2(){
        return startEnd(
                () -> { //extend the intake
                    extendMotor.set(extendSpeed);
                    System.out.println("Intake Extending");
                    SmartDashboard.putBoolean("intakeExtending?", true);
                    SmartDashboard.putBoolean("intakeArmRunning?", true);

                    if(m_bottomlimitSwitch.get() && extendMotor.get() > 0){stopExtension();}
                },
                () -> { //stop extending the intake
                    stopExtension();
                    System.out.println("Extending Stopped");
                    SmartDashboard.putBoolean("intakeExtending?", false);
                    SmartDashboard.putBoolean("intakeArmRunning?", false);
                }
        );
    }

    public Command retractIntake2(){
        return startEnd(
                () -> { //retract the intake
                    extendMotor.set(-extendSpeed);
                    System.out.println("Intake Retracting");
                    SmartDashboard.putBoolean("intakeRetracting?", true);
                    SmartDashboard.putBoolean("intakeArmRunning?", true);

                    if(m_toplimitSwitch.get() && extendMotor.get() < 0){stopExtension();}
                },
                () -> { //stop retracting the intake
                    stopExtension();
                    System.out.println("Extending Stopped");
                    SmartDashboard.putBoolean("intakeRetracting?", false);
                    SmartDashboard.putBoolean("intakeArmRunning?", false);
                }
        );
    }



}
