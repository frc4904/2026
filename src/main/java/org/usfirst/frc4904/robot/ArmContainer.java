package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.Simulation.ArmSimulator;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSimIO;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSubsystem;

public class ArmContainer {
    public static ArmSimulator armsim;
    public static ArmSubsystem arm;
    public ArmContainer(boolean isSim){
            if (isSim) {
                arm = new ArmSubsystem(
                    new ArmSimIO()
                );
            } else {
                arm = null;
            }
                
            armsim = new ArmSimulator(arm);
    }

}
