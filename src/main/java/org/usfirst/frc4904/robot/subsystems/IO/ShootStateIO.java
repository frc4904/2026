package org.usfirst.frc4904.robot.subsystems.IO;

import java.util.Optional;

public class ShootStateIO {
    public record InputState(        
      
        double currentAngularVelocityRadPerSec
        ){}
    
    public record OutputState(
        Optional<Double> voltage
   ){}

    public record GoalState(
        double position
   ){}
}
