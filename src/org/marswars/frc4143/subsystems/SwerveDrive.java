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
    private boolean driveFront = true;
    private double invertFL = 1.;
    private double invertFR = 1.;
    private double invertRL = 1.;
    private double invertRR = 1.;
    private double offsetFL;
    private double offsetFR;
    private double offsetRL;
    private double offsetRR;
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
        
        pidFL.enable();
        pidFR.enable();
        pidRL.enable();
        pidRR.enable();
        
        i2c = DigitalModule.getInstance(1).getI2C(0x04 << 1);
        radius = Math.sqrt(MathUtils.pow(height, 2) + MathUtils.pow(width, 2));
        retrieveOffsets();
    }

    public void retrieveOffsets() {
        offsetFL = Preferences.getInstance().getDouble("FLOffset", offsetFL);
        offsetFR = Preferences.getInstance().getDouble("FROffset", offsetFR);
        offsetRL = Preferences.getInstance().getDouble("RLOffset", offsetRL);
        offsetRR = Preferences.getInstance().getDouble("RROffset", offsetRR);
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
                || Math.abs(potSteerRR.getTurns()) > 5
                || brake < -0.5) {
            setDriveSpeed(0., 0., 0., 0.);
            return;
        }

        double forward = y;
        double strafe = x;

        double xRear = strafe - twist * width / radius;
        double xFront = strafe + twist * width / radius;
        double yRight = forward - twist * height / radius;
        double yLeft = forward + twist * height / radius;

        double setPointFL = 2.5;
        double setPointFR = 2.5;
        double setPointRL = 2.5;
        double setPointRR = 2.5;

        if (Math.abs(yLeft) > 1E-6 || Math.abs(xFront) > 1E-6) {
            setPointFL = (2.5 + 2.5 / Math.PI * MathUtils.atan2(xFront, yLeft));
        }
        if (Math.abs(xFront) > 1E-6 || Math.abs(yRight) > 1E-6) {
            setPointFR = (2.5 + 2.5 / Math.PI * MathUtils.atan2(xFront, yRight));
        }
        if (Math.abs(xRear) > 1E-6 || Math.abs(yLeft) > 1E-6) {
            setPointRL = (2.5 + 2.5 / Math.PI * MathUtils.atan2(xRear, yLeft));
        }
        if (Math.abs(xRear) > 1E-6 || Math.abs(yRight) > 1E-6) {
            setPointRR = (2.5 + 2.5 / Math.PI * MathUtils.atan2(xRear, yRight));
        }
        setSteerSetpoint(setPointFL, setPointFR, setPointRL, setPointRR);

        double speedFL = Math.sqrt(MathUtils.pow(xFront, 2) + MathUtils.pow(yLeft, 2));
        double speedFR = Math.sqrt(MathUtils.pow(xFront, 2) + MathUtils.pow(yRight, 2));
        double speedRL = Math.sqrt(MathUtils.pow(xRear, 2) + MathUtils.pow(yLeft, 2));
        double speedRR = Math.sqrt(MathUtils.pow(xRear, 2) + MathUtils.pow(yRight, 2));

        //Solve for fastest wheel speed
        double speedarray[] = {Math.abs(speedFL), Math.abs(speedFR), Math.abs(speedRL), Math.abs(speedRR)};

        int arrayLength = 4;
        double maxspeed = speedarray[0];
        for (int i = 1; i < arrayLength; i++) {
            maxspeed = Math.max(maxspeed, speedarray[i]);
        }

        //Set ratios based on maximum wheel speed
        if (maxspeed > 1) {
            speedFL = speedFL / maxspeed;
            speedFR = speedFR / maxspeed;
            speedRL = speedRL / maxspeed;
            speedRR = speedRR / maxspeed;
        }

        //Set drive speeds
        setDriveSpeed(speedFL, -speedFR, speedRL, -speedRR);

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
            if (Math.abs(FLSetPoint + offsetFL - potSteerFL.getAverageVoltage()) < 1.25 || Math.abs(FLSetPoint + offsetFL - potSteerFL.getAverageVoltage()) > 3.75) {
                pidFL.setSetpoint(correctSteerSetpoint(FLSetPoint + offsetFL));
                invertFL = 1;
            } else {
                pidFL.setSetpoint(correctSteerSetpoint(FLSetPoint + offsetFL - 2.5));
                invertFL = -1;
            }

            if (Math.abs(FRSetPoint + offsetFR - potSteerFR.getAverageVoltage()) < 1.25 || Math.abs(FRSetPoint + offsetFR - potSteerFR.getAverageVoltage()) > 3.75) {
                pidFR.setSetpoint(correctSteerSetpoint(FRSetPoint + offsetFR));
                invertFR = 1;
            } else {
                pidFR.setSetpoint(correctSteerSetpoint(FRSetPoint + offsetFR - 2.5));
                invertFR = -1;
            }

            if (Math.abs(RLSetPoint + offsetRL - potSteerRL.getAverageVoltage()) < 1.25 || Math.abs(RLSetPoint + offsetRL - potSteerRL.getAverageVoltage()) > 3.75) {
                pidRL.setSetpoint(correctSteerSetpoint(RLSetPoint + offsetRL));
                invertRL = 1;
            } else {
                pidRL.setSetpoint(correctSteerSetpoint(RLSetPoint + offsetRL - 2.5));
                invertRL = -1;
            }

            if (Math.abs(RRSetPoint + offsetRR - potSteerRR.getAverageVoltage()) < 1.25 || Math.abs(RRSetPoint + offsetRR - potSteerRR.getAverageVoltage()) > 3.75) {
                pidRR.setSetpoint(correctSteerSetpoint(RRSetPoint + offsetRR));
                invertRR = 1;
            } else {
                pidRR.setSetpoint(correctSteerSetpoint(RRSetPoint + offsetRR - 2.5));
                invertRR = -1;
            }

        } else {
            if (Math.abs(RRSetPoint + offsetFL - potSteerFL.getAverageVoltage()) < 1.25 || Math.abs(RRSetPoint + offsetFL - potSteerFL.getAverageVoltage()) > 3.75) {
                pidFL.setSetpoint(correctSteerSetpoint(RRSetPoint + offsetFL));
                invertFL = 1;
            } else {
                pidFL.setSetpoint(correctSteerSetpoint(RRSetPoint + offsetFL - 2.5));
                invertFL = -1;
            }

            if (Math.abs(RLSetPoint + offsetFR - potSteerFR.getAverageVoltage()) < 1.25 || Math.abs(RLSetPoint + offsetFR - potSteerFR.getAverageVoltage()) > 3.75) {
                pidFR.setSetpoint(correctSteerSetpoint(RLSetPoint + offsetFR));
                invertFR = 1;
            } else {
                pidFR.setSetpoint(correctSteerSetpoint(RLSetPoint + offsetFR - 2.5));
                invertFR = -1;
            }

            if (Math.abs(FRSetPoint + offsetRL - potSteerRL.getAverageVoltage()) < 1.25 || Math.abs(FRSetPoint + offsetRL - potSteerRL.getAverageVoltage()) > 3.75) {
                pidRL.setSetpoint(correctSteerSetpoint(FRSetPoint + offsetRL));
                invertRL = 1;
            } else {
                pidRL.setSetpoint(correctSteerSetpoint(FRSetPoint + offsetRL - 2.5));
                invertRL = -1;
            }

            if (Math.abs(FLSetPoint + offsetRR - potSteerRR.getAverageVoltage()) < 1.25 || Math.abs(FLSetPoint + offsetRR - potSteerRR.getAverageVoltage()) > 3.75) {
                pidRR.setSetpoint(correctSteerSetpoint(FLSetPoint + offsetRR));
                invertRR = 1;
            } else {
                pidRR.setSetpoint(correctSteerSetpoint(FLSetPoint + offsetRR - 2.5));
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
        retval = unwindWheel(potSteerFL, pidFL)
                || unwindWheel(potSteerFR, pidFR)
                || unwindWheel(potSteerRL, pidRL)
                || unwindWheel(potSteerRR, pidRR);
        isUnwinding = false;
        return retval;
    }
    
    public boolean unwindFL() {
        boolean retval = false;
        isUnwinding = true;
        retval = unwindWheel(potSteerFL, pidFL);
        isUnwinding = false;
        return retval;
    }
    
    public boolean unwindFR() {
        boolean retval = false;
        isUnwinding = true;
        retval = unwindWheel(potSteerFR, pidFR);
        isUnwinding = false;
        return retval;
    }
    
    public boolean unwindRL() {
        boolean retval = false;
        isUnwinding = true;
        retval = unwindWheel(potSteerRL, pidRL);
        isUnwinding = false;
        return retval;
    }
    
    public boolean unwindRR() {
        boolean retval = false;
        isUnwinding = true;
        retval = unwindWheel(potSteerRR, pidRR);
        isUnwinding = false;
        return retval;
    }

    private boolean unwindWheel(AnalogChannelVolt wheel, PIDController pid) {
        double temp;
        double turns = wheel.getTurns();
        if (turns >= 1) {
            temp = wheel.getAverageVoltage() - 1.0;
            if (temp < 0.0) {
                temp = 5.0 + temp;
            }
            pid.setSetpoint(temp);
            return true;
        } else if (turns <= 1) {
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
        offsetFL = FLOff;
        offsetFR = FROff;
        offsetRL = RLOff;
        offsetRR = RROff;
        Preferences.getInstance().putDouble("FLOffset", offsetFL);
        Preferences.getInstance().putDouble("FROffset", offsetFR);
        Preferences.getInstance().putDouble("RLOffset", offsetRL);
        Preferences.getInstance().putDouble("RROffset", offsetRR);
        Preferences.getInstance().save();
    }
}
