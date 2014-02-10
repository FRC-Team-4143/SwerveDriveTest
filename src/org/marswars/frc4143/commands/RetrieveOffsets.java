
package org.marswars.frc4143.commands;

import edu.wpi.first.wpilibj.DriverStation;

/**
 *
 * @author bradmiller
 */
public class RetrieveOffsets extends CommandBase {
    
    private boolean isExecuted = false;

    public RetrieveOffsets() {
        // Use requires() here to declare subsystem dependencies
        requires(swerveDrive);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (!isExecuted) {
            swerveDrive.retrieveOffsets();
            isExecuted = true;
        }
        
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isExecuted;
    }

    // Called once after isFinished returns true
    protected void end() {
        isExecuted = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
