package org.marswars.frc4143;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static final double chassisLength = 100.0; //mm
    public static final double chassisWidthFront = 100.0; //mm
    public static final double chassisWidthRear = 100.0; //mm
    
    public static final boolean CONTINUOUS = true;
    public static final double P = 1.0;
    public static final double I = 0.0;
    public static final double D = 0.2;
    public static final double F = 0.0;
    public static final double POTMIN = 0.0;
    public static final double POTMAX = 5.0;
    public static final double STEERPOW = 0.75;
    public static final double TOLERANCE = 0.2;
    public static final double PERIOD = 0.02;
    
    public static final int portPotSteerFR = 1;
    public static final int portPotSteerRL = 2;
    public static final int portPotSteerRR = 3;
    public static final int portPotSteerFL = 4;
    public static final int portMotorDriveFL = 1;
    public static final int portMotorDriveFR = 2;
    public static final int portMotorDriveRR = 3;
    public static final int portMotorDriveRL = 4;
    public static final int portMotorSteerFR = 5;
    public static final int portMotorSteerRL = 6;
    public static final int portMotorSteerRR = 7;
    public static final int portMotorSteerFL = 8;
}
