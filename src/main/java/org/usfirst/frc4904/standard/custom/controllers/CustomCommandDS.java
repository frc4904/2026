package org.usfirst.frc4904.standard.custom.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class CustomCommandDS extends Joystick {
    protected final int port;
    // Buttons
    public final JoystickButton button1;
    public final JoystickButton button2;
    public final JoystickButton button3;
    public final JoystickButton button4;
    public final JoystickButton button5;
    public final JoystickButton button6;
    public final JoystickButton button7;
    public final JoystickButton button8;
    public final JoystickButton button9;
    public final JoystickButton button10;
    public final JoystickButton button11;
    public final JoystickButton button12;
    public final JoystickButton button13;
    public final JoystickButton button14;
    public final JoystickButton button15;
    public final JoystickButton button16;
    public final JoystickButton button17;
    public final JoystickButton button18;
    public final JoystickButton button19;
    public final JoystickButton button20;
    public final JoystickButton button21;
    public final JoystickButton button22;
    public final JoystickButton button23;
    public final JoystickButton button24;
    public final JoystickButton button25;
    public final JoystickButton button26;
    public final JoystickButton button27;
    public final JoystickButton button28;
    public final JoystickButton button29;
    public final JoystickButton button30;
    public final JoystickButton button31;
    public final JoystickButton button32;

    public CustomCommandDS(int port, double deadzone) {
      super(port);
       this.port = port;
        button1  = new JoystickButton(this, 1);
        button2  = new JoystickButton(this, 2);
        button3  = new JoystickButton(this, 3);
        button4  = new JoystickButton(this, 4);
        button5  = new JoystickButton(this, 5);
        button6  = new JoystickButton(this, 6);
        button7  = new JoystickButton(this, 7);
        button8  = new JoystickButton(this, 8);
        button9  = new JoystickButton(this, 9);
        button10 = new JoystickButton(this, 10);
        button11 = new JoystickButton(this, 11);
        button12 = new JoystickButton(this, 12);
        button13 = new JoystickButton(this, 13);
        button14 = new JoystickButton(this, 14);
        button15 = new JoystickButton(this, 15);
        button16 = new JoystickButton(this, 16);
        button17 = new JoystickButton(this, 17);
        button18 = new JoystickButton(this, 18);
        button19 = new JoystickButton(this, 19);
        button20 = new JoystickButton(this, 20);
        button21 = new JoystickButton(this, 21);
        button22 = new JoystickButton(this, 22);
        button23 = new JoystickButton(this, 23);
        button24 = new JoystickButton(this, 24);
        button25 = new JoystickButton(this, 25);
        button26 = new JoystickButton(this, 26);
        button27 = new JoystickButton(this, 27);
        button28 = new JoystickButton(this, 28);
        button29 = new JoystickButton(this, 29);
        button30 = new JoystickButton(this, 30);
        button31 = new JoystickButton(this, 31);
        button32 = new JoystickButton(this, 32);
 
    }

   /**
     * Returns true if the joystick is actually connected. It determines this by
     * counting the number of buttons (more than 0 means the joystick is connected).
     *
     * @return
     */
    public boolean connected() {
        return DriverStation.getStickButtonCount(port) > 0;
    }
}
