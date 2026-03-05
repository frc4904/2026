package org.usfirst.frc4904.standard.custom.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.usfirst.frc4904.robot.Robot;

/**
 * A joystick that implements the generic controller interface and the 2023
 * trigger interface. This allows us to use a joystick as a controller. This
 * contains 12 buttons to reflect the joysticks we are typically using.
 */
public class CustomCommandPS4 extends CommandPS4Controller {

    // public so that the loop can be used with custom triggers. for example:
    // new Trigger(joystick.loop, () -> joystick.getAxis(Axis.X) > 0.5)
    public final EventLoop loop;

    protected final double deadzone;

    public CustomCommandPS4(int port, double deadzone) {
        super(port);
        if (deadzone < 0 || deadzone >= 1) {
            throw new IllegalArgumentException("Joystick deadzone must be in [0, 1)");
        }
        this.deadzone = deadzone;

        loop = new EventLoop();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(loop::poll);

        Robot.addClearBindingCallback(this::clearBindings);
    }

    @Override
    public Trigger button(int button) {
        return super.button(button, loop);
    }

    /**
     * Not actually "deprecated"; do not remove.
     * Should not be used in our code, use {@link #button(int)} instead.
     */
    @Override @Deprecated
    public Trigger button(int button, EventLoop _loop) {
        // use our loop instead
        return super.button(button, this.loop);
    }

    public Trigger pov(int pov, int angle) {
        return super.pov(pov, angle, loop);
    }

    /**
     * Not actually "deprecated"; do not remove.
     * Should not be used in our code, use other pov methods instead like
     * {@link #pov(int)}, {@link #pov(int, int)}, or {@link #povUp()}.
     */
    @Override @Deprecated
    public Trigger pov(int pov, int angle, EventLoop _loop) {
        return super.pov(pov, angle, this.loop);
    }

    public void clearBindings() {
        loop.clear();
    }

    /**
     * Get the X axis value of left side of the controller.
     */
    @Override
    public double getLeftX() {
        return deadzone(super.getLeftX());
    }

    /**
     * Get the Y axis value of left side of the controller.
     */
    @Override
    public double getLeftY() {
        return deadzone(super.getLeftY());
    }

    /**
     * Get the X axis value of right side of the controller.
     */
    @Override
    public double getRightX() {
        return deadzone(super.getRightX());
    }

    /**
     * Get the Y axis value of right side of the controller.
     */
    @Override
    public double getRightY() {
        return deadzone(super.getRightY());
    }

    protected double deadzone(double input) {
        return MathUtil.applyDeadband(input, deadzone);
    }
}
