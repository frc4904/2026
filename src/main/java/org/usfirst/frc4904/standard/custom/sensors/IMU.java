package org.usfirst.frc4904.standard.custom.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public interface IMU {

    /** Wrap an angle in rotations to the range [0, 1) */
    static double wrapAnglePositive(double angle) {
        angle %= 1;
        return angle < 0 ? angle + 1 : angle;
    }
    /** Wrap an angle in rotations to the range [-1/2, 1/2) */
    static double wrapAngleHalf(double angle) {
        return wrapAnglePositive(angle + 0.5) - 0.5;
    }

    /** @return current yaw in rotations; positive = counterclockwise */
    double getYaw();

    /** @return current pitch in rotations; positive = front up */
    double getPitch();

    /** @return current roll in rotations; positive = left side up, right side down */
    double getRoll();

    /** @return current yaw as a {@link Rotation2d} */
    default Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getYaw());
    }

    /** @return current rotation as a {@link Rotation3d} */
    default Rotation3d getRotation3d() {
        // TODO some of these are probably negated
        return new Rotation3d(
            Units.rotationsToRadians(getRoll()),
            Units.rotationsToRadians(getPitch()),
            Units.rotationsToRadians(getYaw())
        );
    }

    /** Set the zero position to the current yaw */
    default void zeroYaw() {
        System.out.println("reset imu");
        zeroYaw(0);
    }
    /** Set the current yaw to the provided rotation */
    void zeroYaw(double offset);

    /** Get current hardware temperature in freedom units (°F) */
    double getTemperature();

}
