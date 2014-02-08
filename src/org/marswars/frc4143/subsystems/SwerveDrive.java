package org.marswars.frc4143.subsystems;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.marswars.frc4143.AnalogChannelVolt;
import org.marswars.frc4143.RobotMap;
import org.marswars.frc4143.commands.CrabDrive;

/**
 *
 */
public class SwerveDrive extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public AnalogChannelVolt potSteerFL = new AnalogChannelVolt(RobotMap.portPotSteerFL);
    public AnalogChannelVolt potSteerFR = new AnalogChannelVolt(RobotMap.portPotSteerFR);
    public AnalogChannelVolt potSteerRL = new AnalogChannelVolt(RobotMap.portPotSteerRL);
    public AnalogChannelVolt potSteerRR = new AnalogChannelVolt(RobotMap.portPotSteerRR);
    private SpeedController motorSteerFL = new Victor(RobotMap.portMotorSteerFL);
    private SpeedController motorSteerFR = new Victor(RobotMap.portMotorSteerFR);
    private SpeedController motorSteerRL = new Victor(RobotMap.portMotorSteerRL);
    private SpeedController motorSteerRR = new Victor(RobotMap.portMotorSteerRR);
    public PIDController pidFR = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, potSteerFR, motorSteerFR, RobotMap.PERIOD);
    public PIDController pidFL = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, potSteerFL, motorSteerFL, RobotMap.PERIOD);
    public PIDController pidRR = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, potSteerRR, motorSteerRR, RobotMap.PERIOD);
    public PIDController pidRL = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, potSteerRL, motorSteerRL, RobotMap.PERIOD);
    private SpeedController motorDriveFL = new Victor(RobotMap.portMotorDriveFL);
    private SpeedController motorDriveFR = new Victor(RobotMap.portMotorDriveFR);
    private SpeedController motorDriveRL = new Victor(RobotMap.portMotorDriveRL);
    private SpeedController motorDriveRR = new Victor(RobotMap.portMotorDriveRR);
    public I2C i2c;
    private boolean isUnwinding = false;
    private double robotAngle = 0.;
    private double width;
    private double height;
    private double AP;
    private double BP;
    private double CP;
    private double DP;
    private double FL;
    private double FR;
    private double RL;
    private double RR;
    private double FLRatio;
    private double FRRatio;
    private double RLRatio;
    private double RRRatio;
    private boolean driveFront = true;
    private double invertFL = 1.;
    private double invertFR = 1.;
    private double invertRL = 1.;
    private double invertRR = 1.;
    private double FLOffset;
    private double FROffset;
    private double RLOffset;
    private double RROffset;
    private double radius;

    public SwerveDrive() {
        pidFR.setContinuous(RobotMap.CONTINUOUS);
        pidFL.setContinuous(RobotMap.CONTINUOUS);
        pidRR.setContinuous(RobotMap.CONTINUOUS);
        pidRL.setContinuous(RobotMap.CONTINUOUS);

        pidFR.setAbsoluteTolerance(RobotMap.TOLERANCE);
        pidFL.setAbsoluteTolerance(RobotMap.TOLERANCE);
        pidRR.setAbsoluteTolerance(RobotMap.TOLERANCE);
        pidRL.setAbsoluteTolerance(RobotMap.TOLERANCE);

        pidFR.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);
        pidFL.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);
        pidRR.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);
        pidRL.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);

        pidFR.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);
        pidFL.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);
        pidRR.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);
        pidRL.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);

        potSteerFL.start();
        potSteerFR.start();
        potSteerRL.start();
        potSteerRR.start();

        pidFL.enable();
        pidFR.enable();
        pidRL.enable();
        pidRR.enable();

        i2c = DigitalModule.getInstance(1).getI2C(0x04 << 1);
        radius = Math.sqrt(MathUtils.pow(height, 2) + MathUtils.pow(width, 2));
        retrieveOffsets();
    }

    private void retrieveOffsets() {
        FLOffset = Preferences.getInstance().getDouble("FLOffset", FLOffset);
        FROffset = Preferences.getInstance().getDouble("FROffset", FROffset);
        RLOffset = Preferences.getInstance().getDouble("RLOffset", RLOffset);
        RROffset = Preferences.getInstance().getDouble("RROffset", RROffset);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CrabDrive());
    }

    public void toggleFrontBack() {
        driveFront = !driveFront;
        outputLED();
    }

    public void Crab(double twist, double y, double x, double brake) {
        if (isUnwinding
                || Math.abs(potSteerFL.getTurns()) > 5
                || Math.abs(potSteerFR.getTurns()) > 5
                || Math.abs(potSteerRL.getTurns()) > 5
                || Math.abs(potSteerRR.getTurns()) > 5) {
            setDriveSpeed(0., 0., 0., 0.);
            return;
        }

        if (Math.abs(twist) < 1E-6 && Math.abs(y) < 1E-6 && Math.abs(x) < 1E-6) {
            y = 0.05;
        }

        double forward = y;
        double strafe = x;

        AP = strafe - twist * width / radius;
        BP = strafe + twist * width / radius;
        CP = forward - twist * height / radius;
        DP = forward + twist * height / radius;

        double FLSetPoint = 2.5;
        double FRSetPoint = 2.5;
        double RLSetPoint = 2.5;
        double RRSetPoint = 2.5;

        if (Math.abs(DP) > 1E-6 || Math.abs(BP) > 1E-6) {
            FLSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(BP, DP));
        }
        if (Math.abs(BP) > 1E-6 || Math.abs(CP) > 1E-6) {
            FRSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(BP, CP));
        }
        if (Math.abs(AP) > 1E-6 || Math.abs(DP) > 1E-6) {
            RLSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(AP, DP));
        }
        if (Math.abs(AP) > 1E-6 || Math.abs(CP) > 1E-6) {
            RRSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(AP, CP));
        }

        setSteerSetpoint(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);

        FL = Math.sqrt(MathUtils.pow(BP, 2) + MathUtils.pow(DP, 2));
        FR = Math.sqrt(MathUtils.pow(BP, 2) + MathUtils.pow(CP, 2));
        RL = Math.sqrt(MathUtils.pow(AP, 2) + MathUtils.pow(DP, 2));
        RR = Math.sqrt(MathUtils.pow(AP, 2) + MathUtils.pow(CP, 2));

        //Solve for fastest wheel speed
        double speedarray[] = {Math.abs(FL), Math.abs(FR), Math.abs(RL), Math.abs(RR)};

        int arrayLength = 4;
        double maxspeed = speedarray[0];
        for (int i = 1; i < arrayLength; i++) {
            maxspeed = Math.max(maxspeed, speedarray[i]);
        }

        //Set ratios based on maximum wheel speed
        if (maxspeed > 1 || maxspeed < -1) {
            FLRatio = FL / maxspeed;
            FRRatio = FR / maxspeed;
            RLRatio = RL / maxspeed;
            RRRatio = RR / maxspeed;
        } else {
            FLRatio = FL;
            FRRatio = FR;
            RLRatio = RL;
            RRRatio = RR;
        }

        if (brake < -0.5 || y == 0.05 || x == 0.05 || twist == 0.05) {
            FLRatio = 0;
            FRRatio = 0;
            RLRatio = 0;
            RRRatio = 0;
        }

        //Set drive speeds
        setDriveSpeed(FLRatio, -FRRatio, RLRatio, -RRRatio);

    }

    public void Steer(double speed, double angle) {
        // Valid angle input: -Math.PI/2 < angle < Math.PI/2
        // Valid speed input: -1 < speed < 1

        double radiustest = RobotMap.chassisLength / (2 * Math.cos(Math.PI / 2. - Math.abs(angle))); // Law of cosines

    }

    private void setDriveSpeed(double FLSpeed, double FRSpeed, double RLSpeed, double RRSpeed) {
        if (driveFront) {
            motorDriveFL.set(FLSpeed * invertFL);
            motorDriveFR.set(FRSpeed * invertFR);
            motorDriveRL.set(RLSpeed * invertRL);
            motorDriveRR.set(RRSpeed * invertRR);
        } else {
            motorDriveFL.set(RRSpeed * invertFL);
            motorDriveFR.set(RLSpeed * invertFR);
            motorDriveRL.set(FRSpeed * invertRL);
            motorDriveRR.set(FLSpeed * invertRR);
        }
    }

    private void setSteerSetpoint(double FLSetPoint, double FRSetPoint, double RLSetPoint, double RRSetPoint) {
        if (driveFront) {
            if (Math.abs(FLSetPoint + FLOffset - potSteerFL.getAverageVoltage()) < 1.25 || Math.abs(FLSetPoint + FLOffset - potSteerFL.getAverageVoltage()) > 3.75) {
                pidFL.setSetpoint(correctSteerSetpoint(FLSetPoint + FLOffset));
                invertFL = 1;
            } else {
                pidFL.setSetpoint(correctSteerSetpoint(FLSetPoint + FLOffset - 2.5));
                invertFL = -1;
            }

            if (Math.abs(FRSetPoint + FROffset - potSteerFR.getAverageVoltage()) < 1.25 || Math.abs(FRSetPoint + FROffset - potSteerFR.getAverageVoltage()) > 3.75) {
                pidFR.setSetpoint(correctSteerSetpoint(FRSetPoint + FROffset));
                invertFR = 1;
            } else {
                pidFR.setSetpoint(correctSteerSetpoint(FRSetPoint + FROffset - 2.5));
                invertFR = -1;
            }

            if (Math.abs(RLSetPoint + RLOffset - potSteerRL.getAverageVoltage()) < 1.25 || Math.abs(RLSetPoint + RLOffset - potSteerRL.getAverageVoltage()) > 3.75) {
                pidRL.setSetpoint(correctSteerSetpoint(RLSetPoint + RLOffset));
                invertRL = 1;
            } else {
                pidRL.setSetpoint(correctSteerSetpoint(RLSetPoint + RLOffset - 2.5));
                invertRL = -1;
            }

            if (Math.abs(RRSetPoint + RROffset - potSteerRR.getAverageVoltage()) < 1.25 || Math.abs(RRSetPoint + RROffset - potSteerRR.getAverageVoltage()) > 3.75) {
                pidRR.setSetpoint(correctSteerSetpoint(RRSetPoint + RROffset));
                invertRR = 1;
            } else {
                pidRR.setSetpoint(correctSteerSetpoint(RRSetPoint + RROffset - 2.5));
                invertRR = -1;
            }

        } else {

            if (Math.abs(RRSetPoint + FLOffset - potSteerFL.getAverageVoltage()) < 1.25 || Math.abs(RRSetPoint + FLOffset - potSteerFL.getAverageVoltage()) > 3.75) {
                pidFL.setSetpoint(correctSteerSetpoint(RRSetPoint + FLOffset));
                invertFL = 1;
            } else {
                pidFL.setSetpoint(correctSteerSetpoint(RRSetPoint + FLOffset - 2.5));
                invertFL = -1;
            }

            if (Math.abs(RLSetPoint + FROffset - potSteerFR.getAverageVoltage()) < 1.25 || Math.abs(RLSetPoint + FROffset - potSteerFR.getAverageVoltage()) > 3.75) {
                pidFR.setSetpoint(correctSteerSetpoint(RLSetPoint + FROffset));
                invertFR = 1;
            } else {
                pidFR.setSetpoint(correctSteerSetpoint(RLSetPoint + FROffset - 2.5));
                invertFR = -1;
            }

            if (Math.abs(FRSetPoint + RLOffset - potSteerRL.getAverageVoltage()) < 1.25 || Math.abs(FRSetPoint + RLOffset - potSteerRL.getAverageVoltage()) > 3.75) {
                pidRL.setSetpoint(correctSteerSetpoint(FRSetPoint + RLOffset));
                invertRL = 1;
            } else {
                pidRL.setSetpoint(correctSteerSetpoint(FRSetPoint + RLOffset - 2.5));
                invertRL = -1;
            }

            if (Math.abs(FLSetPoint + RROffset - potSteerRR.getAverageVoltage()) < 1.25 || Math.abs(FLSetPoint + RROffset - potSteerRR.getAverageVoltage()) > 3.75) {
                pidRR.setSetpoint(correctSteerSetpoint(FLSetPoint + RROffset));
                invertRR = 1;
            } else {
                pidRR.setSetpoint(correctSteerSetpoint(FLSetPoint + RROffset - 2.5));
                invertRR = -1;
            }
        }
    }

    private double correctSteerSetpoint(double setpoint) {
        while (setpoint < 0 || setpoint > 5) {
            if (setpoint < 0) {
                setpoint = setpoint + 5;
            } else if (setpoint > 5) {
                setpoint = setpoint - 5;
            } else if (setpoint == 5) {
                return 0;
            }
        }
        return setpoint;
    }

    public void outputLED() {
        i2c.write(0x0, 40 * (driveFront ? 1 : 0));
    }

    public void setWheelbase(double newWidth, double newHeight) {
        width = newWidth;
        height = newHeight;
        radius = Math.sqrt(MathUtils.pow(height, 2) + MathUtils.pow(width, 2));
    }

    public boolean unwind() {
        boolean retval = false;
        isUnwinding = true;
        retval = unwindWheel(potSteerFL, pidFL) || unwindWheel(potSteerFR, pidFR) || unwindWheel(potSteerRL, pidRL) || unwindWheel(potSteerRR, pidRR);
        if (!retval) {
            isUnwinding = false;
        }
        return retval;
    }

    private boolean unwindWheel(AnalogChannelVolt wheel, PIDController pid) {
        double temp;
        double turns = wheel.getTurns();
        if (turns > 1) {
            temp = wheel.getAverageVoltage() - 1.0;
            if (temp < 0.0) {
                temp = 5.0 + temp;
            }
            pid.setSetpoint(temp);
            return true;
        } else if (turns < 1) {
            temp = wheel.getAverageVoltage() + 1.0;
            if (temp > 5.0) {
                temp = temp - 5.0;
            }
            pid.setSetpoint(temp);
            return true;
        } else {
            return false;
        }
    }

    public void doneUnwind() {
        isUnwinding = false;
    }

    public void angleUp() {
        robotAngle += 0.1;
        if (robotAngle > 360.) {
            robotAngle = 0.;
        }
        outputLED();
    }

    public void angleDown() {
        robotAngle -= 0.1;
        if (robotAngle > 360.) {
            robotAngle = 0.;
        }
        outputLED();
    }

    public boolean resetTurns() {
        potSteerFR.resetTurns();
        potSteerFL.resetTurns();
        potSteerRR.resetTurns();
        potSteerRL.resetTurns();
        robotAngle = 0.;
        return true;

    }

    public void setOffsets(double FLOff, double FROff, double RLOff, double RROff) {
        FLOffset = FLOff;
        FROffset = FROff;
        RLOffset = RLOff;
        RROffset = RROff;
        Preferences.getInstance().putDouble("FLOffset", FLOffset);
        Preferences.getInstance().putDouble("FROffset", FROffset);
        Preferences.getInstance().putDouble("RLOffset", RLOffset);
        Preferences.getInstance().putDouble("RROffset", RROffset);
        Preferences.getInstance().save();
    }
}
