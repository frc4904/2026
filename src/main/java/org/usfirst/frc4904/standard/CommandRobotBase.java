package org.usfirst.frc4904.standard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.CommandSendableChooser;
import org.usfirst.frc4904.standard.custom.NamedSendableChooser;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.CmdUtil;

import java.util.HashSet;
import java.util.Set;

/**
 * IterativeRobot is normally the base class for command based code, but we
 * think certain features will almost always be needed, so we created the
 * CommandRobotBase class. Robot should extend this instead of iterative robot.
 */
public abstract class CommandRobotBase extends TimedRobot {

    /// CHOOSERS

    private Command autonCommand;
    private Driver driver;
    private Operator operator;

    protected CommandSendableChooser autonChooser;
    protected SendableChooser<Driver> driverChooser;
    protected SendableChooser<Operator> operatorChooser;

    private void displayChoosers() {
        SmartDashboard.putData("chooser/auton", autonChooser);
        SmartDashboard.putData("chooser/driver", driverChooser);
        SmartDashboard.putData("chooser/operator", operatorChooser);
    }

    private void updateHumanInput(Driver driver) {
        updateHumanInput(driver, operator);
    }

    private void updateHumanInput(Operator operator) {
        updateHumanInput(driver, operator);
    }

    private void updateHumanInput(Driver driver, Operator operator) {
        clearBindings();

        this.driver = driver;
        this.operator = operator;

        if (driver != null) driver.bindCommands();
        if (operator != null) operator.bindCommands();
    }

    /// BINDINGS

    private static final Set<Runnable> clearBindingCallbacks = new HashSet<>();

    /**
     * Add a callback that will be called to unbind commands from a joystick/controller
     */
    public static void addClearBindingCallback(Runnable callback) {
        clearBindingCallbacks.add(callback);
    }

    private void clearBindings() {
        if (driver != null) driver.unbindCommands();
        if (operator != null) operator.unbindCommands();

        for (var callback : clearBindingCallbacks) {
            callback.run();
        }
    }

    /// LIFECYCLE CODE

    // The following methods are 'final' to prevent accidentally overriding robot
    // functionality when implementing year specific code. Instead, use the methods
    // linked in the Javadoc for each method below.

    /** Use {@link #initialize()} for year-specific code. */
    @Override
    public final void robotInit() {
        RobotMap.initialize();

        autonChooser = new CommandSendableChooser();
        driverChooser = new NamedSendableChooser<>();
        operatorChooser = new NamedSendableChooser<>();

        initialize();
        displayChoosers();

        updateHumanInput(driverChooser.getSelected(), operatorChooser.getSelected());

        driverChooser.onChange(this::updateHumanInput);
        operatorChooser.onChange(this::updateHumanInput);

        if (driver == null) {
            System.err.println("No default driver was set. Make sure to run driverChooser.setDefaultOption() in Robot.initialize()");
        }
        if (operator == null) {
            System.err.println("No default operator was set. Make sure to run operatorChooser.setDefaultOption() in Robot.initialize()");
        }
    }

    /** Use {@link #alwaysExecute()} for year-specific code. */
    @Override
    public void robotPeriodic() {
        alwaysExecute();
    }

    /** Use {@link #teleopInitialize()} for year-specific code. */
    @Override
    public final void teleopInit() {
        teleopInitialize();
    }

    /** Use {@link #teleopExecute()} for year-specific code. */
    @Override
    public final void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        teleopExecute();
    }

    /** Use {@link #teleopCleanup()} for year-specific code. */
    @Override
    public final void teleopExit() {
        teleopCleanup();
    }

    /** Use {@link #autonomousInitialize()} for year-specific code. */
    @Override
    public final void autonomousInit() {
        autonCommand = autonChooser.getSelected();
        if (autonCommand != null) {
            CmdUtil.schedule(autonCommand);
        }

        autonomousInitialize();
    }

    /** Use {@link #autonomousExecute()} for year-specific code. */
    @Override
    public final void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        autonomousExecute();
    }

    /** Use {@link #autonomousCleanup()} for year-specific code. */
    @Override
    public final void autonomousExit() {
        if (autonCommand != null) {
            autonCommand.cancel();
        }

        autonomousCleanup();
    }

    /** Use {@link #disabledInitialize()} for year-specific code. */
    @Override
    public final void disabledInit() {
        disabledInitialize();
    }

    /** Use {@link #disabledExecute()} for year-specific code. */
    @Override
    public final void disabledPeriodic() {
        CommandScheduler.getInstance().run();
        disabledExecute();
    }

    /** Use {@link #disabledCleanup()} for year-specific code. */
    @Override
    public final void disabledExit() {
        disabledCleanup();
    }

    /** Use {@link #testInitialize()} for year-specific code. */
    @Override
    public final void testInit() {
        testInitialize();
    }

    /** Use {@link #testExecute()} for year-specific code. */
    @Override
    public final void testPeriodic() {
        CommandScheduler.getInstance().run();
        testExecute();
    }

    /** Use {@link #testCleanup()} for year-specific code. */
    @Override
    public final void testExit() {
        testCleanup();
    }


    /**
     * Function for year-specific code to be run on robot code launch, after {@link RobotMap#initialize() RobotMap initialization}.
     * Driver/operator/auton chooser options should be added here.
     */
    public abstract void initialize();

    /** Function for year-specific code to be run in every robot mode. */
    public abstract void alwaysExecute();


    /** Function for year-specific code to be run on teleoperated initialize. */
    public abstract void teleopInitialize();

    /** Function for year-specific code to be run during teleoperated time. */
    public abstract void teleopExecute();

    /** Function for year-specific code to be run when teleoperated mode ends. */
    public abstract void teleopCleanup();


    /** Function for year-specific code to be run on autonomous initialize. */
    public abstract void autonomousInitialize();

    /** Function for year-specific code to be run during autonomous. */
    public abstract void autonomousExecute();

    /** Function for year-specific code to be run when autonomous mode ends. */
    public abstract void autonomousCleanup();


    /** Function for year-specific code to be run on disabled initialize. */
    public abstract void disabledInitialize();

    /** Function for year-specific code to be run while disabled. */
    public abstract void disabledExecute();

    /** Function for year-specific code to be run when disabled mode ends. */
    public abstract void disabledCleanup();


    /** Function for year-specific code to be run on disabled initialize. */
    public abstract void testInitialize();

    /** Function for year-specific code to be run while in test mode. */
    public abstract void testExecute();

    /** Function for year-specific code to be run when test mode ends. */
    public abstract void testCleanup();

}
