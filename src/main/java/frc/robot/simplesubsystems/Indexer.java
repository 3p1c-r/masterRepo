package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemConstants.*;

/* 
 * A simple flywheel indexer. Works in tandem with the shooter. 
 * Based on 4015's 2026 indexer. 
 * See 2026 kitbot design for a somewhat accurate representation of the shooter & indexer classes.
 * Was designed to shoot fuel in REBUILT 2026.
 * 
 * THIS DOESN'T ONLY HAVE TO BE AN INDEXER! It can be any subsystem that needs purely rotational motion.
 */

public class Indexer extends SubsystemBase{

    //Declare motors, speeds, default hood direction, and limit switches
    private final PWMSparkMax indexerMotor;
    private final double indexerSpeed;

    //Initialize the indexer
    public Indexer(){
        this.indexerMotor = new PWMSparkMax(indexerMotorPort); //Constants File
        this.indexerSpeed = ConstIndexerSpeed; //Constants File
    }

    //Run the indexer
    public Command runIndexer2(){
        return startEnd(
                () -> {
                    indexerMotor.set(indexerSpeed);
                    System.out.println("Indexer Command Called");
                    SmartDashboard.putBoolean("indexerRunning?", true);
                },
                () -> {
                    indexerMotor.set(0);
                    System.out.println("Indexer Command Stopped");
                    SmartDashboard.putBoolean("indexerRunning?", false);
                }
        );
    }
}
