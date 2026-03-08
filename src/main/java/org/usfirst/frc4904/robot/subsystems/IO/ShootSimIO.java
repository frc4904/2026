package org.usfirst.frc4904.robot.subsystems.IO;



import org.usfirst.frc4904.robot.subsystems.IO.ShootStateIO.InputState;
import org.usfirst.frc4904.robot.subsystems.IO.ShootStateIO.OutputState;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShootSimIO implements ShootIO{
    //public motor
 // public MotorController []  spinners = new MotorController[] {DCMotor.getKrakenX44(2) };
   // private final FlywheelSim sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(2), 0.005, 1), DCMotor.getKrakenX44(2), null);
    private final FlywheelSim sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(CustomTalonFX, 0.005, 1), DCMotor.getKrakenX44(2), null);
    @Override
    public InputState getInstance() {
       
       sim.update(0.02);
        return new ShootStateIO.InputState(0
    
        );
    }

    @Override 
    public void setState(OutputState output) {
        output.voltage().ifPresent((Volts) -> {
        sim.setInputVoltage(Volts);
        });
    }

}
