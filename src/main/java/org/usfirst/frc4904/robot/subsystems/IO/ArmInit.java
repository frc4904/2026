package org.usfirst.frc4904.robot.subsystems.IO;

import org.usfirst.frc4904.robot.Simulation.ArmSimulator;

public class ArmInit {
    public static ArmSimulator armsim;
    public static ArmSubsystem arm;
    public ArmInit(boolean IsSim) {
        if (IsSim) {
                arm = new ArmSubsystem(
                    new ArmSimIO()
                );
            } else {
                arm = null;
            }
                
            armsim = new ArmSimulator(arm);
    }  
    }

