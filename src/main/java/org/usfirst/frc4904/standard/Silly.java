package org.usfirst.frc4904.standard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;

import java.time.LocalDate;
import java.time.Month;
import java.time.Year;

/** Silly shenanigans that are separated to not crowd the main Robot class */
public final class Silly {

    private static final String odometerDistanceKey = "odometer-distance";
    private static double odometerDistance;
    private static Translation2d lastPos;

    private static final String progressKey = "progress", progressCompletedKey = "progress-completed";
    private static double progress, progressChance = 0.1;
    private static int progressCompleted;

    // i wonder whose birthday this is
    private static final LocalDate birthday = LocalDate.of(Year.now().getValue(), Month.SEPTEMBER, 26);

    public static void initialize() {
        // odometer
        odometerDistance = Preferences.getDouble(odometerDistanceKey, 0);

        // progress
        progress = Preferences.getDouble(progressKey, 0);
        progressCompleted = Preferences.getInt(progressCompletedKey, 0);

        // birthday
        SmartDashboard.putBoolean("birthday", LocalDate.now().equals(birthday));
    }

    public static void periodic() {
        // odometer
        SwerveSubsystem chassis = Component.chassis;

        if (chassis.poseEstimatorEnabled()) {
            Translation2d pos = chassis.getPositionEstimate();
            if (lastPos != null) {
                odometerDistance += pos.getDistance(lastPos);
                Preferences.setDouble(odometerDistanceKey, odometerDistance);
            }
            lastPos = pos;
        } else {
            lastPos = null;
        }

        // progress
        if (Math.random() < progressChance) {
            progress += Math.random();
            progressChance -= 0.02;

            if (progress >= 99.95) { // rounds to 100
                progress = 0;
                Preferences.setInt(progressCompletedKey, ++progressCompleted);
            }
            Preferences.setDouble(progressKey, progress);
            SmartDashboard.putString("progress", String.format("%.1f%% (%d done)", progress, progressCompleted));
        } else {
            progressChance += 0.00001;
        }
    }

}
