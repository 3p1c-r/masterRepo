package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemConstants.*;

/* 
 * A simple flywheel shooter. Works in tandem with the indexer. 
 * Based on 4015's 2026 shooter. 
 * See 2026 kitbot design for a somewhat accurate representation of the shooter & indexer classes.
 * Was designed to shoot fuel in REBUILT 2026.
 * 
 * THIS DOESN'T ONLY HAVE TO BE A SHOOTER! It can be any subsystem that needs purely rotational motion.

 */

public class Shooter extends SubsystemBase{

    //Declare motors, speeds, default hood direction, and limit switches
    private final PWMSparkMax shooterMotor;
    private final double shooterSpeed;

    //Initialize the shooter
    public Shooter(){
        this.shooterMotor = new PWMSparkMax(shooterMotorPort); //Constants File
        this.shooterSpeed = ConstShooterSpeed; //Constants File
    }

    //Run the shooter
    public Command runShooter2(){
        return startEnd(
                () -> {
                    shooterMotor.set(shooterSpeed);
                    System.out.println("Shooter Command Called");
                    SmartDashboard.putBoolean("shooterRunning?", true);
                },
                () -> {
                    shooterMotor.set(0);
                    System.out.println("Shooter Command Stopped");
                    SmartDashboard.putBoolean("shooterRunning?", false);
                }
        );
    }

}
