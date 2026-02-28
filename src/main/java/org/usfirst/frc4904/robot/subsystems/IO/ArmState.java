package org.usfirst.frc4904.robot.subsystems.IO;

import java.util.Optional;

public class ArmState {
    public record InputState(        
        double currentAngleDegrees,
        double currentVelocityDegreesPerSecond
        ){}
    
   public record OutputState(
        Optional<Double> voltage
   ){}

   public record GoalState(
        double position
   ){}
   //Put in goal states
}
//Whatever the arm state
