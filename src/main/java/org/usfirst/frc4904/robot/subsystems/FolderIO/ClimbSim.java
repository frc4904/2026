package org.usfirst.frc4904.robot.subsystems.FolderIO;

import static edu.wpi.first.units.Units.Volts;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.Simulation.ClimbSimulator;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimbState.InputState;
import org.usfirst.frc4904.robot.subsystems.IO.ArmState;
import org.usfirst.frc4904.robot.subsystems.IO.ArmState.OutputState;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimbSim implements ClimbIO{
    private final ElevatorSim climbersim =
    //TODO: Find out wtf plant is
        new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), 2, 5, 3), 
        DCMotor.getKrakenX60(1), 
        5, 
        50, 
        true, 
        0, 
        0, 0);
    

    @Override
    public ClimbState.InputState getInstance() {
        climbersim.update(0.02);
        return new ClimbState.InputState(
            climbersim.getPositionMeters(),
            climbersim.getVelocityMetersPerSecond()
            );
    }

    @Override
    public void setClimbState(ClimbState.OutputState output) {
        output.voltage().ifPresent((Volts) -> {
            climbersim.setInputVoltage(Volts);
        });
    }

}
