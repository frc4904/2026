package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4904.robot.subsystems.*;
import org.usfirst.frc4904.robot.swerve.SwerveModule;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.robot.vision.VisionSubsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.custom.sensors.CustomDutyCycleEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CustomPigeon;
import org.usfirst.frc4904.standard.custom.sensors.IMU;
import org.usfirst.frc4904.standard.custom.sensors.LinearDutyCycleEncoder;

public final class RobotMap {

    public static class Port {

        public static class HumanInput {

            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;

            public static final int joystick = 2;

        }

        public static class CANMotor {}

        public static class PWM {

            public static final int LED_STRIP = 9;

        }
    }

    public static class Component {

        public static IMU imu;

        // subsystems
        public static SwerveSubsystem chassis;
        public static LightSubsystem lights;
        public static VisionSubsystem vision;
        public static ClimberSubsystem climber;
        public static IntakeSubsystem intake;
        public static ShooterSubsystem shooter;
        public static MotorSubsystem indexer;
        public static MotorSubsystem TEMPORARY_INTAKE_SHENANIGANS;

        // motors
        public static CustomTalonFX flDrive;
        public static CustomTalonFX flTurn;
        public static CustomTalonFX frDrive;
        public static CustomTalonFX frTurn;
        public static CustomTalonFX blDrive;
        public static CustomTalonFX blTurn;
        public static CustomTalonFX brDrive;
        public static CustomTalonFX brTurn;

        public static SmartMotorController climbMotor;
        public static SmartMotorController intakeVerticalMotor;
        public static SmartMotorController intakeRollerMotor;
        public static SmartMotorController indexerMotorBottom;
        public static SmartMotorController indexerMotorTop;
        public static CustomTalonFX shooterMotorRight;
        public static CustomTalonFX shooterMotorLeft;

        // misc
        public static AddressableLED ledStrip;

        public static PowerDistribution pdh;

        // encoders
        public static DutyCycleEncoder intakeEncoder;
        public static LinearDutyCycleEncoder climberEncoder;


    }

    public static class Input {}

    public static class HumanInput {

        public static class Driver {

            public static CustomCommandXbox xbox;
            public static CustomCommandJoystick xyJoystick;
            public static CustomCommandJoystick turnJoystick;

        }

        public static class Operator {

            public static CustomCommandJoystick joystick;

        }
    }

    public static class Dashboard {

        public static final Field2d liveField = new Field2d();
        public static final Field2d previewField = new Field2d();

        static {
            SmartDashboard.putData("field/live", liveField);
            SmartDashboard.putData("field/preview", previewField);
        }

    }

    private static boolean initialized = false;

    public static void initialize() {
        if (initialized) {
            System.err.println("Robot already initialized");
            return;
        }
        initialized = true;

        Component.pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

        Component.imu = new CustomPigeon(59);

        Component.chassis = new SwerveSubsystem(
            new SwerveModule(
                "Front Left",
                Component.flDrive = new CustomTalonFX(3),
                Component.flTurn = new CustomTalonFX(15),
                new CustomDutyCycleEncoder(0),
                new Translation2d(1, 1)
            ),
            new SwerveModule(
                "Front Right",
                Component.frDrive = new CustomTalonFX(2),
                Component.frTurn = new CustomTalonFX(17),
                new CustomDutyCycleEncoder(1),
                new Translation2d(1, -1)
            ),
            new SwerveModule(
                "Back Left",
                Component.blDrive = new CustomTalonFX(10),
                Component.blTurn = new CustomTalonFX(16),
                new CustomDutyCycleEncoder(2),
                new Translation2d(-1, 1)
            ),
            new SwerveModule(
                "Back Right",
                Component.brDrive = new CustomTalonFX(4),
                Component.brTurn = new CustomTalonFX(18),
                new CustomDutyCycleEncoder(3),
                new Translation2d(-1, -1)
            )
        );

        Component.vision = new VisionSubsystem(
            new Transform2d(Units.inchesToMeters(8), Units.inchesToMeters(0), Rotation2d.kZero)
        );

        // Component.ledStrip = new AddressableLED(Port.PWM.LED_STRIP);
        // Component.lights = new LightSubsystem(
        //     Component.ledStrip,
        //     107,
        //     new int[] { 20, 37, 34, 16 },
        //     new boolean[] { false, true, false, true }
        // );

        Component.climbMotor = new CustomTalonFX(1, true);
        Component.climberEncoder = new LinearDutyCycleEncoder(9);

        Component.intakeVerticalMotor = new CustomTalonFX(22);
        Component.intakeRollerMotor = new CustomTalonFX(21, true);
        Component.intakeEncoder = new DutyCycleEncoder(8);

        Component.shooterMotorLeft = new CustomTalonFX(23);
        Component.shooterMotorRight = new CustomTalonFX(24, true);

        Component.indexerMotorBottom = new CustomTalonFX(42);
        Component.indexerMotorTop = new CustomTalonFX(41, true);

        Component.TEMPORARY_INTAKE_SHENANIGANS = new MotorSubsystem(
            Component.intakeVerticalMotor,
            2
        );

        Component.intake = new IntakeSubsystem(Component.intakeVerticalMotor, Component.intakeRollerMotor, Component.intakeEncoder);
        Component.climber = new ClimberSubsystem(Component.climbMotor, Component.climberEncoder);
        Component.shooter = new ShooterSubsystem(Component.shooterMotorRight, Component.shooterMotorLeft);
        Component.indexer = new MotorSubsystem(
            new SmartMotorController[] { Component.indexerMotorBottom },
            0.5
        );


        HumanInput.Driver.xyJoystick = new CustomCommandJoystick(
            Port.HumanInput.xyJoystickPort,
            0.0
        );
        HumanInput.Driver.turnJoystick = new CustomCommandJoystick(
            Port.HumanInput.zJoystickPort,
            0.0
        );
        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
    }

    private RobotMap() {}

}
