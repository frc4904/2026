package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import org.usfirst.frc4904.robot.Simulation.MechanismSimulator;
import org.usfirst.frc4904.robot.subsystems.ClimberSubsystem;
import org.usfirst.frc4904.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc4904.robot.subsystems.LightSubsystem;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSimIO;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSubsystem;
import org.usfirst.frc4904.robot.swerve.SwerveModule;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.robot.vision.VisionSubsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.custom.sensors.CustomDutyCycleEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CustomNavx;

public final class RobotMap {
     
    public static class Port {

        public static class HumanInput {

            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;

            public static final int joystick = 2;
        }

        public static class CANMotor {

        }

        public static class PWM {

            public static final int ENCODER_FL = 0;
            public static final int ENCODER_FR = 1;
            public static final int ENCODER_BL = 2;
            public static final int ENCODER_BR = 3;

            public static final int LED_STRIP = 9;
        }
    }

    public static class Component {

        public static CustomNavx navx;

        // subsystems
        public static SwerveSubsystem chassis;
        public static LightSubsystem lights;
        public static VisionSubsystem vision;
        public static ClimberSubsystem climber;
        public static IntakeSubsystem intake;

      
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

        // misc
        public static AddressableLED ledStrip;

        // encoders
        public static DutyCycleEncoder intakeEncoder;
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

    private static boolean initialized = false;

    public static void initialize() {
        if (initialized) {
            System.err.println("Robot already initialized");
            return;
        }
        initialized = true;

        Component.navx = new CustomNavx(0);

        Component.chassis = new SwerveSubsystem(
            new SwerveModule(
                "Front Left",
                Component.flDrive = new CustomTalonFX(2),
                Component.flTurn = new CustomTalonFX(17),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_FL),
                new Translation2d(1, 1)
            ),
            new SwerveModule(
                "Front Right",
                Component.frDrive = new CustomTalonFX(3),
                Component.frTurn = new CustomTalonFX(15),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_FR),
                new Translation2d(1, -1)
            ),
            new SwerveModule(
                "Back Left",
                Component.blDrive = new CustomTalonFX(4),
                Component.blTurn = new CustomTalonFX(18),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_BL),
                new Translation2d(-1, 1)
            ),
            new SwerveModule(
                "Back Right",
                Component.brDrive = new CustomTalonFX(1),
                Component.brTurn = new CustomTalonFX(16),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_BR),
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

        // Component.climbMotor = new CustomTalonFX(32);

        // Component.intakeVerticalMotor = new CustomTalonFX(17);
        // Component.intakeRollerMotor = new CustomTalonFX(2);
        // Component.intakeEncoder = new DutyCycleEncoder(Port.PWM.ENCODER_FL);

        // Component.intake = new IntakeSubsystem(Component.intakeVerticalMotor, Component.intakeRollerMotor, Component.intakeEncoder);

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
