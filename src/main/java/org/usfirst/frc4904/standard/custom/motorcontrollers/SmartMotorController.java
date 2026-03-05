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

	default SmartMotorController withSpeedMultiplier(double multiplier) {
		return new SpeedModifiedSmartMotor(this, multiplier);
	}

	class SpeedModifiedSmartMotor extends SmartMotorWrapper {
		private final double multiplier;

		public SpeedModifiedSmartMotor(SmartMotorController motor, double multiplier) {
			super(motor);
			this.multiplier = multiplier;
		}

		@Override
		public SmartMotorController withSpeedMultiplier(double multiplier) {
			return new SpeedModifiedSmartMotor(motor, multiplier * this.multiplier);
		}

		@Override
		public void set(double speed) {
			super.set(speed * multiplier);
		}

		@Override
		public double get() {
			return super.get() / multiplier;
		}
	}
}

abstract class SmartMotorWrapper implements SmartMotorController {
	protected final SmartMotorController motor;

	protected SmartMotorWrapper(SmartMotorController motor) {
		this.motor = motor;
	}

	@Override
	public boolean isFwdLimitSwitchPressed() throws IllegalAccessException {
		return motor.isFwdLimitSwitchPressed();
	}
	@Override
	public boolean isRevLimitSwitchPressed() throws IllegalAccessException {
		return motor.isRevLimitSwitchPressed();
	}

	@Override public void setMotorBrake(boolean brake) { motor.setMotorBrake(brake); }

	@Override public void set(double speed) { motor.set(speed); }
	@Override public double get() { return motor.get(); }

	@Override public void setInverted(boolean isInverted) { motor.setInverted(isInverted); }
	@Override public boolean getInverted() { return motor.getInverted(); }

	@Override public void disable() { motor.disable(); }

	@Override public void stopMotor() { motor.stopMotor(); }
}
