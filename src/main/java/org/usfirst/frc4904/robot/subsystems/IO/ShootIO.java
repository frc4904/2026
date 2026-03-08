package org.usfirst.frc4904.robot.subsystems.IO;

import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public interface ShootIO {
    
    
    public ShootStateIO.InputState getInstance();
    public void setState(ShootStateIO.OutputState output); 
}
