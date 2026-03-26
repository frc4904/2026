package org.usfirst.frc4904.robot.Simulation;

import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimbState;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimberTwo;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbSimulator {
    private final Mechanism2d panel;
    private final MechanismRoot2d climbroot;
    private final MechanismLigament2d climber;

    private ClimberTwo climb;    

    public ClimbSimulator(ClimberTwo climb) {
        this.climb = climb;
    

    this.panel = new Mechanism2d(
        Units.inchesToMeters(100), 
        Units.inchesToMeters(100));
    
    this.climbroot = panel.getRoot(
            "climber",
             Units.inchesToMeters(7.35),
             Units.inchesToMeters(10) );

    this.climber = climbroot.append(
            new MechanismLigament2d("climber", 
            Units.inchesToMeters(30), 0, 6, new Color8Bit(Color.kLightGoldenrodYellow))
        );
    }

    public void periodic() {
        ClimbState.InputState currentClimbState = ClimberTwo.getState();
            if (currentClimbState != null) {
                this.climber.setLength(currentClimbState.currentPositionMeters());
            }
            SmartDashboard.putData("Climber Simulation", panel);
    
    
    }
}
