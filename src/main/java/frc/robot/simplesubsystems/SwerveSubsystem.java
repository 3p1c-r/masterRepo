package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Meter;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {

    double maximumSpeed = 5; //Declaring the max speed in meters/second.
    SwerveDrive swerveDrive; //Declaring our swerve drive object that represents the robot.
    RobotConfig config; //For autonomous

    private final CommandPS4Controller driverCtrl = new CommandPS4Controller(OperatorConstants.kDriverControllerPort); //Declaring our PS4 controller and setting it to the port declared in OperatorConstants.

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault(); //Creating a NetworkTable Instance. NetworkTables is the software used to publish telemetry (info abt the robot) to a dashboard.
    private final NetworkTable table = inst.getTable("SmartDashboard/swerve"); //Getting the table representing the swerve drive
    DoubleArraySubscriber measuredChassisSpeedsSub = table.getDoubleArrayTopic("measuredChassisSpeeds").subscribe(new double[] {}); //Reading the value of measuredChassisSpeeds inside of the swerve table.
    double[] measuredChassisSpeeds = measuredChassisSpeedsSub.get(); //Converting the subscriber value to a regular double array.

    public SwerveSubsystem(File directory) { //SwerveSubsystem Constructor
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH; //There are different levels of information we can receive about the robot. We want the highest level of information.

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, new Pose2d(new Translation2d(Meter.of(5),Meter.of(5)), Rotation2d.fromDegrees(90))); //Create the swerve drive at a specified position with the given json files.
        //yeah
        } catch (IOException e) { //If the json files are missing or incomplete...
            throw new RuntimeException("Swerve Drive directory not found. Finish those config files!"); //...throw an error.
        }

        /*for pathplanner reference
        swerveDrive.zeroGyro();
        swerveDrive.getPose();
        swerveDrive.getRobotVelocity();*/

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        AutoBuilder.configure(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }
    //Gets the swerve drive object representing the robot.
    public SwerveDrive getSwerveDrive(){
        return swerveDrive;
    }
    //Drive (robot oriented) with a given ChassisSpeeds (containing X and Y, and Theta (rotation) coordinate details.
    public void drive(ChassisSpeeds velocity){ // This specific method is used for auto.
        swerveDrive.drive(velocity); //Drive the robot.
        System.out.println("DRIVING: " + velocity);
    }

    /*
    Drive (robot oriented) with a given ChassisSpeeds (containing X and Y, and Theta (rotation) coordinate details.
    This method returns a Command to drive (with a lambda expression) so it can be set as a default command in RobotContainer.java.
    Otherwise, this method would not be able to be used because it can't be used for the drivetrain command in RobotContainer.java,
    since WPILIB requires commands to bind to subsystems. */
    public Command drive(Supplier<ChassisSpeeds> velocity){ //Drive robot oriented in Teleop.
        return run(
            () -> {
                //Drive the robot with the ChassisSpeeds supplier (SwerveInputStream in RobotContainer.java.
                swerveDrive.drive(velocity.get());
                System.out.println("DRIVING: " + velocity.get());

                //Publish telemetry: The robot pose and the joystick inputs.
                SmartDashboard.putString("Pose2D", swerveDrive.getPose().toString());
                SmartDashboard.putString("leftXJoystick", String.format("%.2f", driverCtrl.getLeftX()));
                SmartDashboard.putString("leftYJoystick", String.format("%.2f", driverCtrl.getLeftY()));
                SmartDashboard.putString("rightXJoystick", String.format("%.2f", driverCtrl.getRightX()));

                //Publish all the ChassisSpeeds measurements (xSpeed, ySpeed, rotational speed) as separate widgets.
                for(int i = 0; i < measuredChassisSpeeds.length; i++){
                    measuredChassisSpeeds[i] = Math.round(measuredChassisSpeeds[i] * 100.0) / 100.0;

                    if(i == 0){SmartDashboard.putNumber("HorizontalSpeedX", measuredChassisSpeeds[i]);} 
                    else if(i == 1){SmartDashboard.putNumber("VerticalSpeedY", measuredChassisSpeeds[i]);}
                    else {SmartDashboard.putNumber("rotSpeedTheta", measuredChassisSpeeds[i]);}
                }

                SmartDashboard.putNumberArray("RoundedMeasuredChassisSpeeds", measuredChassisSpeeds);

            }
        );
    }

    /*
    Drive (FIELD oriented) with a given ChassisSpeeds (containing X and Y, and Theta (rotation) coordinate details.
    This method returns a Command to drive (with a lambda expression) so it can be set as a default command in RobotContainer.java.
    Otherwise, this method would not be able to be used because it can't be used for the drivetrain command in RobotContainer.java,
    since WPILIB requires commands to bind to subsystems. */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){//Drive field oriented. This is the most used & useful method.
        return run(
            () -> {
                swerveDrive.driveFieldOriented(velocity.get());

                SmartDashboard.putString("leftXJoystick", String.format("%.2f", driverCtrl.getLeftX()));
                SmartDashboard.putString("leftYJoystick", String.format("%.2f", driverCtrl.getLeftY()));
                SmartDashboard.putString("rightXJoystick", String.format("%.2f", driverCtrl.getRightX()));

                double[] measuredChassisSpeeds = measuredChassisSpeedsSub.get();

                for(int i = 0; i < measuredChassisSpeeds.length; i++){
                    measuredChassisSpeeds[i] = Math.round(measuredChassisSpeeds[i] * 100.0) / 100.0;

                    if(i == 0){SmartDashboard.putNumber("HorizontalSpeedX", measuredChassisSpeeds[i]);} 
                    else if(i == 1){SmartDashboard.putNumber("VerticalSpeedY", measuredChassisSpeeds[i]);}
                    else {SmartDashboard.putNumber("rotSpeedTheta", measuredChassisSpeeds[i]);}
                }

                SmartDashboard.putNumberArray("RoundedMeasuredChassisSpeeds", measuredChassisSpeeds);
        });    
    }
}