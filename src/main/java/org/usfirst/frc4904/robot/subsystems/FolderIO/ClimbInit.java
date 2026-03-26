package org.usfirst.frc4904.robot.subsystems.FolderIO;

import org.usfirst.frc4904.robot.Simulation.ClimbSimulator;

public class ClimbInit {
    private ClimberTwo climber;
    public static ClimbSimulator climbsim;

    public ClimbInit (boolean isSim) {
        if (isSim) {
            climber = new ClimberTwo(
                null, 
                null, 
                new ClimbSim()
            );
            } else {
                climber = null;
            }

        climbsim = new ClimbSimulator(climber);     
    }

}
