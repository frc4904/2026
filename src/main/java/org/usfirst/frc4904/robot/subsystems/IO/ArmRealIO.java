package org.usfirst.frc4904.robot.subsystems.IO;

import org.usfirst.frc4904.robot.subsystems.IO.ArmState.InputState;
import org.usfirst.frc4904.robot.subsystems.IO.ArmState.OutputState;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ArmRealIO implements ArmIO{

    Spark spark = new Spark(1);
    //Encoder encoder

    @Override
    public InputState getInstance() {
        //get encoder value return here
        return null;
    }

    @Override
    public void setState(OutputState output) {
        //spark.setVolatge(output.voltage());
    }

}
//Defining the real inputs and outputs