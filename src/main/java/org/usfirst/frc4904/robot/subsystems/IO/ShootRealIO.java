package org.usfirst.frc4904.robot.subsystems.IO;

import org.usfirst.frc4904.robot.subsystems.IO.ArmState.InputState;
import org.usfirst.frc4904.robot.subsystems.IO.ArmState.OutputState;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ShootRealIO implements ShootIO{
    Spark spark = new Spark(1);
    CustomTalonFX shooterMotor1 = new CustomTalonFX(0);
    CustomTalonFX shooterMotor2 = new CustomTalonFX(0);
    //Encoder encoder

    @Override
    public org.usfirst.frc4904.robot.subsystems.IO.ShootStateIO.InputState getInstance() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
    }

    @Override
    public void setState(org.usfirst.frc4904.robot.subsystems.IO.ShootStateIO.OutputState output) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setState'");
    }

}