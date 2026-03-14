package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class IndexerSubsystem extends MotorSubsystem {

    public IndexerSubsystem(SmartMotorController top, SmartMotorController bottom) {
        super(
            new SmartMotorController[] {
                top.withSpeedMultiplier(SmartDashboard.getNumber("indexer/top mult", 1)),
                bottom.withSpeedMultiplier(SmartDashboard.getNumber("indexer/bottom mult", 1))
            },
            SmartDashboard.getNumber("indexer/both volts", 5)
        );

        SmartDashboard.putData("indexer", this);
    }

    private static final double
        DIP_DURATION = 0.2, // duration of the dip in voltage
        DIP_GAP = 0.8, // duration of the gap between dips
        SMOOTHNESS = 1, // smoothness of the dip, in (0, 1] - 0 = square, 1 = sine
                        // as this value gets lower, the dip gets smaller than the declared duration above
        DIP_PERCENT = 0.5; // percentage decrease in voltage, in [0, 1]

    private double topMult = 1;
    private double bottomMult = 1;

    public Command c_pulse() {
        return new Command() {
            double startTime;

            @Override
            public void initialize() {
                startTime = Timer.getFPGATimestamp();
            }

            @Override
            public void execute() {
                setVoltage(forwardVoltage * getMult());
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }

            private double getMult() {
                // https://www.desmos.com/calculator/cujlhpi476

                double t = Timer.getFPGATimestamp() - startTime;
                t %= DIP_DURATION + DIP_GAP;
                if (t >= DIP_DURATION) return 1;

                double cos = Math.cos(2 * Math.PI * t / DIP_DURATION);
                double smoothed = MathUtil.copyDirectionPow(cos, SMOOTHNESS);
                return (smoothed - 1) * DIP_PERCENT / 2 + 1;
            }
        };
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "both volts",
            () -> forwardVoltage,
            (v) -> forwardVoltage = backwardVoltage = v
        );

        builder.addDoubleProperty(
            "top mult",
            () -> topMult,
            (m) -> {
                motors[0] = motors[0].withSpeedMultiplier(topMult = m);
            }
        );

        builder.addDoubleProperty(
            "bottom mult",
            () -> bottomMult,
            (m) -> {
                motors[1] = motors[1].withSpeedMultiplier(bottomMult = m);
            }
        );
    }
}
