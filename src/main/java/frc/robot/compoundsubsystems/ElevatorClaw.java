package frc.robot.compoundsubsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//there's no practical use for this, we can just have a separate elevator and claw,
//but it's just an example of a compound subsystem as a placeholder
public class ElevatorClaw {
    public Elevator elevator = new Elevator();
    public Claw claw = new Claw();

    //Elevator Methods
    public void moveManual(double speed){elevator.moveManual(speed);}
    public Command goToBottom(){return elevator.goToBottom();}
    public Command goToLow(){return elevator.goToLow();}
    public Command goToMid(){return elevator.goToMid();}
    public Command goToHigh(){return elevator.goToHigh();}

    //Claw Methods
    public Command openClaw(){return claw.openClaw();}
    public Command closeClaw(){return claw.closeClaw();}

}

class Elevator extends SubsystemBase {
    //-----------------------------------------------
    // PID CONSTANTS
    //-----------------------------------------------
    private static double kDt = 0.02;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kP = 1.3;
    private static double kI = 0.0;
    private static double kD = 0.7;
    private static double kS = 1.1;
    private static double kG = 1.2;
    private static double kV = 1.3;
    //-----------------------------------------------
    // SPEED LIMITS
    //-----------------------------------------------
    private static final double MAX_MANUAL_SPEED_LIMIT = 0.7;
    private static final double kMaxOutput =  0.8;
    private static final double kMinOutput = -0.8;

    double sprocketDiameter = 3; //inches
    double gearRatio = 5;

    double distancePerRotation = (sprocketDiameter * Math.PI) / gearRatio; //inches
    //-----------------------------------------------
    // POSITIONS
    //-----------------------------------------------
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    // Preset positions in encoder rotations — tune for your robot
    private static final double POSITION_BOTTOM = 0.0;
    private static final double POSITION_LOW    = 11.25;
    private static final double POSITION_MID    = 22.5;
    private static final double POSITION_HIGH   = 45.0;

    // How close to target counts as "at position"
    private static final double POSITION_TOLERANCE = 0.5;

    private double m_targetPosition = POSITION_BOTTOM;

    //-----------------------------------------------
    // HARDWARE
    //-----------------------------------------------
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;

    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    private final SparkRelativeEncoderSim m_encoderSim;
    private final DIOSim m_bottomLimitSim;
    private final DIOSim m_topLimitSim;

    // Create a PID controller whose setpoint's change is subject to maximum// velocity and acceleration constraints.
    private final SparkClosedLoopController controller;

    SparkMaxConfig config = new SparkMaxConfig();

    public Elevator() {
        m_motor = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0.1);
        bottomLimitSwitch = new DigitalInput(10);
        topLimitSwitch = new DigitalInput(11);

        m_encoderSim    = new SparkRelativeEncoderSim(m_motor);
        m_bottomLimitSim = new DIOSim(bottomLimitSwitch);
        m_topLimitSim    = new DIOSim(topLimitSwitch);

// DIOSim defaults to false, but NC switches read true when unpressed
// so initialize them to match your isAtBottom()/isAtTop() logic
        m_bottomLimitSim.setValue(true); // true = not pressed (NC wiring)
        m_topLimitSim.setValue(true);

        controller = m_motor.getClosedLoopController();

        //config
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kP,kI,kD)
                .outputRange(kMinOutput,kMaxOutput);

        config
                .encoder
                .positionConversionFactor(distancePerRotation)
                .velocityConversionFactor(distancePerRotation/60);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ---------------------------------------------------------------------------
    // PID position control
    // ---------------------------------------------------------------------------

    private boolean m_pidMode = false;

    /** Move to an arbitrary position using closed-loop PID. */
    public void setPosition(double position) {
        m_targetPosition = position;
        System.out.println(m_targetPosition);
        controller.setSetpoint(m_targetPosition, SparkBase.ControlType.kPosition);
    }


    public void setPidMode(boolean pidMode) { m_pidMode = pidMode; }
    public boolean isPidMode() { return m_pidMode; }

    /** Convenience methods for named presets. */
    public Command goToBottom() {return setPosTemplate(POSITION_BOTTOM);}
    public Command goToLow()    {return setPosTemplate(POSITION_LOW);}
    public Command goToMid()    {return setPosTemplate(POSITION_MID);}
    public Command goToHigh()   {return setPosTemplate(POSITION_HIGH);}

    public Command setPosTemplate(double position) {
        return run(() -> {
            setPosition(position); setPidMode(true);
            SmartDashboard.putNumber("Elevator Speed", m_motor.get());
            SmartDashboard.putNumber("Elevator Position",      getPosition());
            SmartDashboard.putNumber("Elevator Target",        m_targetPosition);
            SmartDashboard.putBoolean("Elevator At Target",    isAtTarget());
            SmartDashboard.putBoolean("Elevator Bottom Limit", isAtBottom());
            SmartDashboard.putBoolean("Elevator Top Limit",    isAtTop());
            SmartDashboard.putBoolean("ElevatorPIDModeOn?", isPidMode());
        });
    }

    // ---------------------------------------------------------------------------
    // Manual control
    // ---------------------------------------------------------------------------


    public void moveManual(double speed){
        SmartDashboard.putNumber("Elevator Speed", speed);
        SmartDashboard.putNumber("Elevator Position",      getPosition());
        SmartDashboard.putNumber("Elevator Target",        m_targetPosition);
        SmartDashboard.putBoolean("Elevator At Target",    isAtTarget());
        SmartDashboard.putBoolean("Elevator Bottom Limit", isAtBottom());
        SmartDashboard.putBoolean("Elevator Top Limit",    isAtTop());
        SmartDashboard.putBoolean("ElevatorPIDModeOn?", isPidMode());

        m_motor.set(speed);
        m_targetPosition = getPosition();

        if((isAtBottom() && speed < 0) || (isAtTop() && speed > 0)){stop();return;}
        if (isAtBottom()) {resetEncoder();}
    }

    public void stop(){m_motor.set(0);}

    // ---------------------------------------------------------------------------
    // Sensors & states
    // ---------------------------------------------------------------------------

    public boolean isAtBottom(){return bottomLimitSwitch.get();}

    public boolean isAtTop(){return topLimitSwitch.get();}

    public double getPosition(){return m_encoder.getPosition();}

    public double getTargetPosition(){return m_targetPosition;}

    public void resetEncoder() {m_encoder.setPosition(0);}

    /** Returns true when the elevator is within tolerance of the target. */
    public boolean isAtTarget() {return Math.abs(getPosition() - m_targetPosition) < POSITION_TOLERANCE;}

    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    // ---------------------------------------------------------------------------
    // Periodic
    // ---------------------------------------------------------------------------


    @Override
    public void simulationPeriodic() {
        double motorOutput;
        System.out.println(m_pidMode);
        if (m_pidMode) {
            // Manually simulate proportional control for position
            double error = m_targetPosition - getPosition();
            motorOutput = MathUtil.clamp(error * kP * 0.05, kMinOutput, kMaxOutput);
        } else {motorOutput = m_motor.get();}

        double rotationsPerSecond = motorOutput * 60; //Tune this number to match the elevator
        m_encoderSim.iterate(rotationsPerSecond, 0.02);
        // Simulate bottom limit switch — triggers when encoder near zero
        if (getPosition() <= POSITION_BOTTOM + POSITION_TOLERANCE  && m_motor.get() <= 0) {
            m_bottomLimitSim.setValue(true); // false = pressed (NC wiring)
        } else {m_bottomLimitSim.setValue(false);}

        // Simulate top limit switch — triggers when encoder near max
        if (getPosition() >= POSITION_HIGH - POSITION_TOLERANCE && m_motor.get() >= 0) {
            m_topLimitSim.setValue(true); // false = pressed (NC wiring)
        } else {m_topLimitSim.setValue(false);}
    }
}

class Claw extends SubsystemBase {
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


