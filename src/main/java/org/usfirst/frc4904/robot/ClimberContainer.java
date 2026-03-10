package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.Simulation.ArmSimulator;
import org.usfirst.frc4904.robot.Simulation.ClimbSimulator;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimbSim;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimberTwo;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSimIO;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSubsystem;

public class ClimberContainer {
        public static ClimbSimulator climbsim;
            public static ClimberTwo climber;

            public ClimberContainer(boolean isSim){
                if (isSim) {
                    climber = new ClimberTwo(null, null, new ClimbSim());
                } else {
                    climber = null;
                }

                climbsim = new ClimbSimulator(climber);
            }
            }
