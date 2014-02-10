package org.marswars.frc4143;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.marswars.frc4143.commands.AngleDown;
import org.marswars.frc4143.commands.AngleUp;
import org.marswars.frc4143.commands.ResetTurns;
import org.marswars.frc4143.commands.SMDB;
import org.marswars.frc4143.commands.SetWheelOffsets;
import org.marswars.frc4143.commands.ToggleLock;
import org.marswars.frc4143.commands.ToggleRobotFront;
import org.marswars.frc4143.commands.UnwindFL;
import org.marswars.frc4143.commands.UnwindFR;
import org.marswars.frc4143.commands.UnwindRL;
import org.marswars.frc4143.commands.UnwindRR;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static final double deadZone = 0.15;
    private XboxController xbox = new XboxController(1);

    public OI() {
        new JoystickButton(xbox, XboxController.ButtonType.kStart.value).whileHeld(new SMDB());
        new JoystickButton(xbox, XboxController.ButtonType.kY.value).whenPressed(new ToggleRobotFront());
        new JoystickButton(xbox, XboxController.ButtonType.kX.value).whileHeld(new ToggleLock());
        new JoystickButton(xbox, XboxController.ButtonType.kL.value).whileHeld(new AngleUp());
        new JoystickButton(xbox, XboxController.ButtonType.kR.value).whileHeld(new AngleDown());
        SmartDashboard.putData("SetWheelOffsets", new SetWheelOffsets());
        SmartDashboard.putData("ResetTurns", new ResetTurns());
        SmartDashboard.putData("SMDB", new SMDB());
        SmartDashboard.putData("Unwind FL", new UnwindFL());
        SmartDashboard.putData("Unwind FR", new UnwindFR());
        SmartDashboard.putData("Unwind RL", new UnwindRL());
        SmartDashboard.putData("Unwind RR", new UnwindRR());


    }

//1 - LeftX
//2 - LeftY
//3 - Triggers (Each trigger = 0 to 1, axis value = right - left)
//4 - RightX
//5 - RightY
//6 - DPad Left/Right

    public double getJoystickLeftX() {
        if (Math.abs(xbox.getRawAxis(1)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(1);
        }
    }

    public double getJoystickLeftY() {
        if (Math.abs(xbox.getRawAxis(2)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(2);
        }
    }

    public double getJoystickRightX() {
        if (Math.abs(xbox.getRawAxis(4)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(4);
        }
    }

    public double getTriggers() {
        if (Math.abs(xbox.getRawAxis(3)) < deadZone) {
            return 0;
        } else {
            return xbox.getRawAxis(3);
        }
    }
    
    public double getDPadX() {
        return xbox.getRawAxis(6);
    }
}
