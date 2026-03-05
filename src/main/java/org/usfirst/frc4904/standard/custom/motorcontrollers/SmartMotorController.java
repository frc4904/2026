package org.usfirst.frc4904.standard.custom.motorcontrollers;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Represents a "smart" motor controller, like the TalonFX, TalonSRX, or SparkMax
 * <p>
 * These should support brake mode, follow mode, limit switches, and various closed-loop control modes.
 */
public interface SmartMotorController extends MotorController {
	boolean isFwdLimitSwitchPressed() throws IllegalAccessException;
	boolean isRevLimitSwitchPressed() throws IllegalAccessException;

	void setMotorBrake(boolean brake);
}
