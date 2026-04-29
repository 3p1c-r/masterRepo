// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@SuppressWarnings("PMD.RedundantFieldInitializer")
public class Elevator extends SubsystemBase {
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

    double distancePerRotation = (sprocketDiameter * Math.PI) / gearRatio;
    //-----------------------------------------------
    // POSITIONS
    //-----------------------------------------------
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    // Preset positions in encoder rotations — tune for your robot
    private static final double POSITION_BOTTOM = 0.0;
    private static final double POSITION_LOW    = 10.0;
    private static final double POSITION_MID    = 25.0;
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

        m_motor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    // ---------------------------------------------------------------------------
    // PID position control
    // ---------------------------------------------------------------------------

    /** Move to an arbitrary position using closed-loop PID. */
    public void setPosition(double position) {
        m_targetPosition = position;
        controller.setSetpoint(position, SparkBase.ControlType.kPosition);
    }

    /** Convenience methods for named presets. */
    public void goToBottom() { setPosition(POSITION_BOTTOM); }
    public void goToLow()    { setPosition(POSITION_LOW);    }
    public void goToMid()    { setPosition(POSITION_MID);    }
    public void goToHigh()   { setPosition(POSITION_HIGH);   }

    // ---------------------------------------------------------------------------
    // Manual control
    // ---------------------------------------------------------------------------

    public Command moveManual(DoubleSupplier speed){

        return run(
                () -> {
                    if((isAtBottom() && speed.getAsDouble() < 0) || (isAtTop() && speed.getAsDouble() > 0)){
                        stop();
                    }

                    m_motor.set(speed.getAsDouble());
                    m_targetPosition = getPosition();

                    if (isAtBottom()) {
                        resetEncoder();
                    }


                    SmartDashboard.putNumber("Elevator Speed", speed.getAsDouble());
                    SmartDashboard.putNumber("Elevator Position",      getPosition());
                    SmartDashboard.putNumber("Elevator Target",        m_targetPosition);
                    SmartDashboard.putBoolean("Elevator At Target",    isAtTarget());
                    SmartDashboard.putBoolean("Elevator Bottom Limit", isAtBottom());
                    SmartDashboard.putBoolean("Elevator Top Limit",    isAtTop());

                    System.out.println("elevator moving, speed is: " + speed.getAsDouble());
                }
        );

    }

    public void moveManual(double speed){
        SmartDashboard.putNumber("Elevator Speed", speed);
        SmartDashboard.putNumber("Elevator Position",      getPosition());
        SmartDashboard.putNumber("Elevator Target",        m_targetPosition);
        SmartDashboard.putBoolean("Elevator At Target",    isAtTarget());
        SmartDashboard.putBoolean("Elevator Bottom Limit", isAtBottom());
        SmartDashboard.putBoolean("Elevator Top Limit",    isAtTop());

        m_motor.set(speed);
        m_targetPosition = getPosition();

        if((isAtBottom() && speed < 0) || (isAtTop() && speed > 0)){
            stop();
            return;
        }

        if (isAtBottom()) {
            resetEncoder();
        }
    }

    public void stop(){
        m_motor.set(0);
    }

    // ---------------------------------------------------------------------------
    // Sensors & states
    // ---------------------------------------------------------------------------

    public boolean isAtBottom(){
        return bottomLimitSwitch.get();
    }

    public boolean isAtTop(){
        return topLimitSwitch.get();
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    public void resetEncoder() {
        m_encoder.setPosition(0);
    }

    /** Returns true when the elevator is within tolerance of the target. */
    public boolean isAtTarget() {
        return Math.abs(getPosition() - m_targetPosition) < POSITION_TOLERANCE;
    }

    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    // ---------------------------------------------------------------------------
    // Periodic
    // ---------------------------------------------------------------------------
/*
    @Override
    public void periodic() {
        // Keep encoder zeroed whenever the bottom limit is hit
        if (isAtBottom()) {
            resetEncoder();
        }

        SmartDashboard.putNumber("Elevator Position",      getPosition());
        SmartDashboard.putNumber("Elevator Target",        m_targetPosition);
        SmartDashboard.putBoolean("Elevator At Target",    isAtTarget());
        SmartDashboard.putBoolean("Elevator Bottom Limit", isAtBottom());
        SmartDashboard.putBoolean("Elevator Top Limit",    isAtTop());
    } */

    @Override
    public void simulationPeriodic() {
        double rotationsPerSecond = m_motor.get() * 60; // tune 20.0 to match your elevator
        m_encoderSim.iterate(rotationsPerSecond, 0.02);
        // Simulate bottom limit switch — triggers when encoder near zero
        if (getPosition() <= 0.05 && m_motor.get() <= 0) {
            m_bottomLimitSim.setValue(true); // false = pressed (NC wiring)
        } else {
            m_bottomLimitSim.setValue(false);
        }

        // Simulate top limit switch — triggers when encoder near max
        if (getPosition() >= POSITION_HIGH - POSITION_TOLERANCE && m_motor.get() >= 0) {
            m_topLimitSim.setValue(true); // false = pressed (NC wiring)
        } else {
            m_topLimitSim.setValue(false);
        }
    }
}