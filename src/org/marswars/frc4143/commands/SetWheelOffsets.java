package org.marswars.frc4143.commands;

/**
 *
 * @author bradmiller
 */
public class SetWheelOffsets extends CommandBase {
    
    private boolean m_Executed = false;

    public SetWheelOffsets() {
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double FLOffset = swerveDrive.potSteerFL.getAverageVoltage() - 2.5;
        double FROffset = swerveDrive.potSteerFR.getAverageVoltage() - 2.5;
        double RLOffset = swerveDrive.potSteerRL.getAverageVoltage() - 2.5;
        double RROffset = swerveDrive.potSteerRR.getAverageVoltage() - 2.5;

        swerveDrive.setOffsets(FLOffset, FROffset, RLOffset, RROffset);
        m_Executed = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return m_Executed;
    }

    // Called once after isFinished returns true
    protected void end() {
        m_Executed = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
