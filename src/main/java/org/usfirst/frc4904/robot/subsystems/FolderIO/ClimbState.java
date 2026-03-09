package org.usfirst.frc4904.robot.subsystems.FolderIO;

import java.util.Optional;

public class ClimbState {
    public record InputState(        
        double currentPositionMeters, 
        double currentVelocityMetersPerSecond
        ){}
    
   public record OutputState(
        Optional<Double> voltage
   ){}

   public record GoalState(
        double position
   ){}
}
