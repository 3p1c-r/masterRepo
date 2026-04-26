package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemConstants.*;


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
