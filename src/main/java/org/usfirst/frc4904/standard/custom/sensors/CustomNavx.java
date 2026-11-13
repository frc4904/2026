package org.usfirst.frc4904.standard.custom.sensors;

import com.studica.frc.Navx;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;

import org.usfirst.frc4904.robot.RobotMap.Component;

/**
 * NavX should be mounted to robot according to WPILib coordinates:
 * +X is forward, +Y is left, +Z is up
 */
public class CustomNavx implements IMU {

    protected final Navx navx;
    private double yawOffset;
    private double lastYaw;

    /**
     * Creates a new NavX
     *
     * @param id the CAN id of the navx.
     */
    public CustomNavx(int id) {
        navx = new Navx(id);
    }

    @Override
    public double getYaw() {
        double raw = navx.getYaw().in(Rotations);
        System.out.println("raw: " + raw);

        if (raw == 1) return lastYaw;

        return lastYaw = IMU.wrapAnglePositive(raw + yawOffset);
    }
    @Override
    public double getPitch() {
        return -navx.getRoll().in(Rotations);
    }
    @Override
    public double getRoll() {
        return navx.getPitch().in(Rotations);
    }

    @Override
    public void zeroYaw(double offset) {
        yawOffset = offset; 
        navx.resetYaw();
    }

    @Override
    public double getTemperature() {
        return navx.getTemperature().in(Fahrenheit);
    }

}
