package org.usfirst.frc4904.robot.subsystems.IO;

import org.usfirst.frc4904.robot.subsystems.IO.ArmState.InputState;
import org.usfirst.frc4904.robot.subsystems.IO.ArmState.OutputState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSimIO implements ArmIO{
//TODO: Find all the parameters for actual motors we use
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getNEO(1), 
        50, 
        SingleJointedArmSim.estimateMOI(Units.inchesToMeters(25), Units.lbsToKilograms(20)),
        Units.inchesToMeters(25),
        -2 * Math.PI,
        2 * Math.PI, 
        true, 0, 0, 0
    );

    @Override
    public InputState getInstance() {
        sim.update(0.02);
        return new ArmState.InputState(
            Units.radiansToDegrees(sim.getAngleRads()),
            Units.radiansToDegrees(sim.getVelocityRadPerSec())
            );
    }

    @Override
    public void setState(OutputState output) {
        output.voltage().ifPresent((Volts) -> {
            sim.setInputVoltage(Volts);
        });
    }

}
//Info for the sim to make inputs and outputs