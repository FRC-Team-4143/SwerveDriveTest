package org.marswars.frc4143;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.marswars.frc4143.commands.AngleDown;
import org.marswars.frc4143.commands.AngleUp;
import org.marswars.frc4143.commands.SMDB;
import org.marswars.frc4143.commands.ToggleLock;
import org.marswars.frc4143.commands.ToggleRobotFront;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static final double deadZone = 0.1;
    private XboxController xbox = new XboxController(1);

    public OI() {
        new JoystickButton(xbox, XboxController.ButtonType.kStart.value).whileHeld(new SMDB());
        new JoystickButton(xbox, XboxController.ButtonType.kY.value).whenPressed(new ToggleRobotFront());
        new JoystickButton(xbox, XboxController.ButtonType.kX.value).whileHeld(new ToggleLock());
        new JoystickButton(xbox, XboxController.ButtonType.kL.value).whileHeld(new AngleUp());
        new JoystickButton(xbox, XboxController.ButtonType.kR.value).whileHeld(new AngleDown());



    }
    //SmartDashboard.putData("SetWheelOffsets", new SetWheelOffsets());
    //SmartDashboard.putData("ResetGyro", new ResetGyro());
    //SmartDashboard.putData("ResetTurns", new ResetTurns());

    public double getJoystickX() {
        if (Math.abs(xbox.getRawAxis(1)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(1);
        }
    }

    public double getJoystickY() {
        if (Math.abs(xbox.getRawAxis(2)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(2);
        }
    }

    public double getJoystickZ() {
        if (Math.abs(xbox.getRawAxis(4)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(4);
        }
    }

    public double getJoystickA() {
        if (Math.abs(xbox.getRawAxis(3)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(3);
        }
    }
}
