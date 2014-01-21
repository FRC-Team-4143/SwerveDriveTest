package org.marswars.frc4143.commands;

import edu.wpi.first.wpilibj.DriverStation;

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
        double FLOffset = swerveDrive.potSteerFL.getAverageVoltage() - 2.5;
        double FROffset = swerveDrive.potSteerFR.getAverageVoltage() - 2.5;
        double RLOffset = swerveDrive.potSteerRL.getAverageVoltage() - 2.5;
        double RROffset = swerveDrive.potSteerRR.getAverageVoltage() - 2.5;

        swerveDrive.setOffsets(FLOffset, FROffset, RLOffset, RROffset);

        m_Executed = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        DriverStation.getInstance().setDigitalOut(1, true);
        DriverStation.getInstance().setDigitalOut(2, true);
        DriverStation.getInstance().setDigitalOut(3, true);
        DriverStation.getInstance().setDigitalOut(4, true);
        DriverStation.getInstance().setDigitalOut(5, true);
        DriverStation.getInstance().setDigitalOut(6, true);
        DriverStation.getInstance().setDigitalOut(7, true);
        DriverStation.getInstance().setDigitalOut(8, true);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return m_Executed;
    }

    // Called once after isFinished returns true
    protected void end() {
        DriverStation.getInstance().setDigitalOut(1, false);
        DriverStation.getInstance().setDigitalOut(2, false);
        DriverStation.getInstance().setDigitalOut(3, false);
        DriverStation.getInstance().setDigitalOut(4, false);
        DriverStation.getInstance().setDigitalOut(5, false);
        DriverStation.getInstance().setDigitalOut(6, false);
        DriverStation.getInstance().setDigitalOut(7, false);
        DriverStation.getInstance().setDigitalOut(8, false);
        m_Executed = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
