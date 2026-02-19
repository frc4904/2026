package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.Simulation.MechanismSimulator;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSimIO;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSubsystem;

public final class SimulationContainer {
        public final MechanismSimulator sim;
        public static ArmSubsystem arm;
        public SimulationContainer(boolean isSim){
            if (isSim) {
                arm = new ArmSubsystem(
                    new ArmSimIO()
                );
            } else {
                arm = null;
            }

            sim = new MechanismSimulator(arm);
        }
}
