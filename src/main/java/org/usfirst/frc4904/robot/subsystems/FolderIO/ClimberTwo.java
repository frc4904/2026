package org.usfirst.frc4904.robot.subsystems.FolderIO;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import org.usfirst.frc4904.robot.subsystems.MotorSubsystem;
import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimbState.GoalState;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.usfirst.frc4904.standard.custom.sensors.LinearDutyCycleEncoder;

public class ClimberTwo extends MotorSubsystem {

        private final ClimbIO io;

    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;

    private static ClimbState.InputState currentState;
    
        private ClimbState.GoalState goal = new ClimbState.GoalState(12);
    
            // TODO change all these
        public static final double kS = 1;
        public static final double kV = 2;
        public static final double kA = 0.4;
        public static final double kG = 0.2;
    
        public static final double kP = 6;
        public static final double kI = 0;
        public static final double kD = 0;
    
        public static final double MAX_VEL = 8;
        public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second
    
        private static final double MAX_HEIGHT = 123456789;
        private static final double MIN_HEIGHT = 0;
    
        private final ElevatorFeedforward feedforward;
        private final LinearDutyCycleEncoder encoder;
                
                    
                
                
                    public ClimberTwo(SmartMotorController climberTwomotor, LinearDutyCycleEncoder climberTwoencoder, ClimbIO climbIO) {
                        super(climberTwomotor, 4);
                
                        this.io= climbIO;
                        this.encoder = climberTwoencoder;
                        this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
                
                        this.controller = new ProfiledPIDController(0.001, 0, 0, new Constraints(100, 100));
                        this.ff = new ElevatorFeedforward(0, 2.6057, 0);
                    }
                
                    public Command c_up() {
                        return c_forward(true);
                    }
                
                    public Command c_down() {
                        return c_backward(true);
                    }
                
                    public double getHeight() {
                        return encoder.get();
        }
    
        public Command c_gotoHeight(double height) {
            ezControl controller = new ezControl(
                kP, kI, kD,
                (position, velocity) -> feedforward.calculate(velocity)
            );
            var constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);
    
            return new ezMotion(
                controller,
                this::getHeight,
                this::setVoltage,
                height,
                constraints,
                this
            ).finallyDo(this::stop);
        }
    
        @Override
        public void setVoltage(double voltage) {
            setVoltage(voltage, false);
        }
    
        public void setVoltage(double voltage, boolean bypassSoftwareStop) {
            if (
                !bypassSoftwareStop && (
                    (this.getHeight() >= MAX_HEIGHT && voltage > 0) ||
                    (this.getHeight() <= MIN_HEIGHT && voltage < 0)
                )
            ) {
                voltage = 0;
            }
            super.setVoltage(voltage);
        }
        
        @Override
        public void periodic() {
            if(DriverStation.isEnabled()) {
                currentState = this.io.getInstance();
    
                double effort = this.controller.calculate(currentState.currentPositionMeters(), goal.position());
                double feedforward = this.ff.calculate(
                    currentState.currentPositionMeters());
                    currentState.currentVelocityMetersPerSecond();
    
                effort += feedforward;
    
                this.io.setClimbState(
                    new ClimbState.OutputState(Optional.of(effort))
                );
            }
        }
    
        public static ClimbState.InputState getState() {
            return currentState;
    }

    public Command setClimbPosition(double position) {
        return new InstantCommand(() -> this.goal = new ClimbState.GoalState(position));
    }

}
