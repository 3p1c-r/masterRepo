package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemConstants.*;


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
