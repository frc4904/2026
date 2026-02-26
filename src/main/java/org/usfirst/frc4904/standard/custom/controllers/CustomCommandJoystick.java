package org.usfirst.frc4904.standard.custom.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A joystick that implements the generic controller interface and the 2023
 * trigger interface. This allows us to use a joystick as a controller. This
 * contains 12 buttons to reflect the joysticks we are typically using.
 */
public class CustomCommandJoystick extends CommandJoystick {

    public enum Axis {
        X(0),
        Y(1),
        TWIST(2),
        SLIDER(3);

        public final int axis;

        Axis(int axis) {
            this.axis = axis;
        }
    }

    private final EventLoop loop;

    protected final double deadzone;
    protected final int port;

    public final Trigger button1;
    public final Trigger button2;
    public final Trigger button3;
    public final Trigger button4;
    public final Trigger button5;
    public final Trigger button6;
    public final Trigger button7;
    public final Trigger button8;
    public final Trigger button9;
    public final Trigger button10;
    public final Trigger button11;
    public final Trigger button12;

    public CustomCommandJoystick(int port, double deadzone) {
        super(port);
        if (deadzone < 0 || deadzone > 1) {
            throw new IllegalArgumentException("Joystick deadzone must be in [0, 1]");
        }
        this.deadzone = deadzone;
        this.port = port;

        loop = new EventLoop();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(loop::poll);

        button1  = button(1);
        button2  = button(2);
        button3  = button(3);
        button4  = button(4);
        button5  = button(5);
        button6  = button(6);
        button7  = button(7);
        button8  = button(8);
        button9  = button(9);
        button10 = button(10);
        button11 = button(11);
        button12 = button(12);
    }

    @Override
    public Trigger button(int id) {
        return super.button(id, loop);
    }

    @Override
    public Trigger button(int button, EventLoop loop) {
        // clearBindings() won't work if the button is on a different event loop
        throw new UnsupportedOperationException("CustomCommandJoystick.button() does not support setting a custom event loop.");
    }

    public void clearBindings() {
        loop.clear();
    }

    /**
     * Returns true if the joystick is actually connected. It determines this by
     * counting the number of buttons (more than 0 means the joystick is connected).
     */
    public boolean connected() {
        return DriverStation.getStickButtonCount(port) > 0;
    }

    /**
     * Returns the raw value of the given axis (no deadzone calculation).
     */
    public double getRawAxis(Axis axis) {
        return super.getRawAxis(axis.axis);
    }

    /**
     * Returns the value of the given axis, with the deadzone applied.
     */
    public double getAxis(Axis axis) {
        return MathUtil.applyDeadband(getRawAxis(axis), deadzone);
    }

    /**
     * Returns true if a given axis is above the move threshold.
     *
     * @param axis
     * @return whether the current value of that axis is outside the deadzone
     */
    public boolean active(Axis axis) {
        return Math.abs(getRawAxis(axis)) > deadzone;
    }
}
