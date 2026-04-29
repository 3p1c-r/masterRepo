package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.map;

public class elevatorCommand extends Command {
    private final Elevator elevator;
    private final CommandPS4Controller driverCtrl = new CommandPS4Controller(Constants.OperatorConstants.kDriverControllerPort);


    public elevatorCommand(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){



    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double R2Mapped = map(driverCtrl.getR2Axis(), -1, 1, 0, 1);
        double L2Mapped = map(driverCtrl.getL2Axis(), -1, 1, 0, 1);
        double speed = R2Mapped - L2Mapped;
        elevator.moveManual(speed);
        System.out.println("L2: " + L2Mapped + " R2: " + R2Mapped + " Speed: " + speed );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.stop();


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}


