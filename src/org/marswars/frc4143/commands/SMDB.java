
package org.marswars.frc4143.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author bradmiller
 */
public class SMDB extends CommandBase {

    public SMDB() {
        // Use requires() here to declare subsystem dependencies
         setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        SmartDashboard.putNumber("FrontLeftVolt", swerveDrive.potSteerFL.getVoltage());
        SmartDashboard.putNumber("FrontRightVolt", swerveDrive.potSteerFR.getVoltage());
        SmartDashboard.putNumber("RearLeftVolt", swerveDrive.potSteerRL.getVoltage());
        SmartDashboard.putNumber("RearRightVolt", swerveDrive.potSteerRR.getVoltage());
        SmartDashboard.putNumber("FrontLeftturns", swerveDrive.potSteerFL.getTurns());
        SmartDashboard.putNumber("FrontRightturns", swerveDrive.potSteerFR.getTurns());
        SmartDashboard.putNumber("RearLeftturns", swerveDrive.potSteerRL.getTurns());
        SmartDashboard.putNumber("RearRightturns", swerveDrive.potSteerRR.getTurns());
        SmartDashboard.putNumber("StickX", oi.getJoystickX());
        SmartDashboard.putNumber("StickY", oi.getJoystickY());
        SmartDashboard.putNumber("StickZ", oi.getJoystickZ());
        SmartDashboard.putData("FLpid", swerveDrive.pidFL);
        SmartDashboard.putNumber("FLError", swerveDrive.pidFL.getError());
        SmartDashboard.putNumber("FLoutput", swerveDrive.pidFL.get());
        SmartDashboard.putData("FRpid", swerveDrive.pidFR);
        SmartDashboard.putNumber("FRError", swerveDrive.pidFR.getError());
        SmartDashboard.putNumber("FRoutput", swerveDrive.pidFR.get());
        SmartDashboard.putData("RLpid", swerveDrive.pidRL);
        SmartDashboard.putNumber("RLError", swerveDrive.pidRL.getError());
        SmartDashboard.putNumber("RLoutput", swerveDrive.pidRL.get());
        SmartDashboard.putData("RRpid", swerveDrive.pidRR);
        SmartDashboard.putNumber("RRError", swerveDrive.pidRR.getError());
        SmartDashboard.putNumber("RRoutput", swerveDrive.pidRR.get());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
