// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics extends SubsystemBase {
    private final Solenoid m_solenoid;
    private final Compressor m_compressor;

    public Pneumatics() {
        m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0); //channel should be the can id port
        m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    }
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command extendSolenoid() {
        return runOnce(
                () -> {
                    m_solenoid.set(true);
                    SmartDashboard.putBoolean("SolenoidExtended?", true);
                    System.out.println("extending");
                });
    }

    public Command retractSolenoid() {
        return runOnce(
                () -> {
                    m_solenoid.set(false);
                    SmartDashboard.putBoolean("SolenoidExtended?", false);
                    System.out.println("retracting");
                });
    }

    public Command toggleSolenoid() {
        return runOnce(
                () -> {
                    m_solenoid.toggle();
                    SmartDashboard.putBoolean("SolenoidExtended?", m_solenoid.get());
                    System.out.println("solenoid extension set to " + m_solenoid.get());
                }
        );
    }
    //not required, purely for reference/simulation
    public Command enableCompressor(){
        return runOnce(() -> {
            m_compressor.enableDigital();
            SmartDashboard.putBoolean("CompressorEnabled?", true);
            System.out.println("compressor enabled");
        });
    }

    public Command killCompressor() {
        return runOnce(
                () -> {
                    m_compressor.disable();
                    SmartDashboard.putBoolean("CompressorEnabled?", false);
                    System.out.println("compressor disabled");
                });
    }


}















// hi :D