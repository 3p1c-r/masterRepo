// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevatorCommand;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  //Hardware defined here:
  private final CommandPS4Controller driverCtrl = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller operatorCtrl = new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...

  //Subsystems
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Indexer indexer = new Indexer();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Elevator elevator = new Elevator();

  //Intake Commands
  Command runIntake = intake.runIntake2();
  Command runOuttake = intake.runOuttake2();
  Command extendIntake = intake.extendIntake2();
  Command retractIntake = intake.retractIntake2();

  //Shooter Commands
  Command runShooter = shooter.runShooter2();
  Command runIndexer = indexer.runIndexer2();

  //Pneumatic Commands
  Command toggleSolenoid = pneumatics.toggleSolenoid();
  Command toggleCompressor = pneumatics.toggleCompressor();

  //Elevator Commands
  elevatorCommand elevatorCMD = new elevatorCommand(elevator);



  //Other Commands
  WaitCommand wait = new WaitCommand(1.25);

  private final PathPlannerAuto auto;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("intake", runIntake);
    NamedCommands.registerCommand("extendIntake", extendIntake);
    NamedCommands.registerCommand("shoot", runShooter);
    NamedCommands.registerCommand("index", runIndexer);

    auto = new PathPlannerAuto("rebuiltAuto");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //Get ChassisSpeeds to drive the robot
  public SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverCtrl.getLeftY() * -1,
                  () -> driverCtrl.getLeftX() * 1)
          .withControllerRotationAxis(() -> -driverCtrl.getRightX())
          .deadband(Constants.OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  public SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverCtrl.getLeftX() * 1,
                  () -> -driverCtrl.getLeftY() * -1)
          .withControllerRotationAxis(() -> -driverCtrl.getRightX())
          .deadband(Constants.OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);
                        
  public SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
                                                    .robotRelative(true)
                                                    .allianceRelativeControl(false);

  //Drivebase Commands
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveFieldOrientedAngularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
  Command driveRobotOrientedAngularVelocity = drivebase.drive(driveRobotOriented);


  private void configureBindings() {

    //Drivebase Bindings
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocitySim);
    driverCtrl.circle().toggleOnTrue(driveRobotOrientedAngularVelocity
            .beforeStarting(() -> SmartDashboard.putBoolean("isRobotOriented", true))
            .finallyDo(() -> SmartDashboard.putBoolean("isRobotOriented", false)));

    //Intake Bindings

    driverCtrl.L1().toggleOnTrue(runIntake);
    driverCtrl.square().toggleOnTrue(runOuttake);

    /*
    driverCtrl.L2().toggleOnTrue(extendIntake);
    driverCtrl.R2().toggleOnTrue(retractIntake); */

    //Shooter/Indexer Bindings
    driverCtrl.R1().toggleOnTrue(runShooter
            .alongWith(
                    wait
                          .beforeStarting(() -> SmartDashboard.putBoolean("Waiting?", true))
                          .finallyDo(() -> SmartDashboard.putBoolean("Waiting?", false))
                          .andThen(runIndexer)
            )
    );

    //Pneumatic Bindings
    driverCtrl.cross().toggleOnTrue(toggleSolenoid);
    driverCtrl.triangle().toggleOnTrue(toggleCompressor);


    //Elevator Bindings
    elevator.setDefaultCommand(elevatorCMD);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    System.out.println("Autonomous Command Called!!!");
    return auto;
  }
}
