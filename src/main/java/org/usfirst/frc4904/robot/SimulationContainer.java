package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.Simulation.ArmSimulator;
import org.usfirst.frc4904.robot.Simulation.ClimbSimulator;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimbSim;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimberTwo;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSimIO;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSubsystem;

public final class SimulationContainer {
    public SimulationContainer() {
        public final ArmSimulator armsim;
                public static ArmSubsystem arm;
            public final class ArmContainer {
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

        public static ClimbSimulator climbsim;
            public static ClimberTwo climber;
        public final class ClimbContainer {
            public ClimbContainer(boolean isSim){
                if (isSim) {
                    climber = new ClimberTwo(null, null, new ClimbSim());
                } else {
                    climber = null;
                }

                climbsim = new ClimbSimulator(climber);
            }
            }
        }
}
