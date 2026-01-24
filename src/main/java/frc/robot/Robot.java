// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class SwerveWheel {
    TalonFX speedMotor;
    double currSpeed;

    TalonFX dirMotor;
    PIDController dirController;

    public SwerveWheel(TalonFX speedMotor, TalonFX dirMotor,
                       double p, double i, double d) {
        this.speedMotor = speedMotor;
        this.currSpeed = 0;

        this.dirMotor = dirMotor;
        this.dirController = new PIDController(p, i, d);
        // Causes the PIDController to treat -180 and 180 as the same point for
        // automatically calculating the shortest route to the setpoint.
        this.dirController.enableContinuousInput(-180, 180);
    }

    private double currMotorAngle() {
        StatusSignal<Angle> signal = dirMotor.getPosition();
        Angle angle = signal.getValue();

        // TODO: Need to apply some conversion factor to this to get an angle in degrees.

        // TODO: Need to use encoder to offset this so that the angle we report is the
        // direction the wheel is actually pointing, and some angle relative to the
        // wheel's position at startup.

        return angle.baseUnitMagnitude();
    }

    public void setDirection(double desiredAngle) {
        desiredAngle = Utils.normalizeAngle(desiredAngle);
        dirController.reset();
        dirController.setSetpoint(desiredAngle);
    }

    public void setSpeed(double speed) {
        currSpeed = speed;
    }

    public void periodic() {
        speedMotor.set(currSpeed);
        double dirMotorPower = dirController.calculate(currMotorAngle());
        dirMotor.set(dirMotorPower);
    }
}

class SwerveDrive {
    // Shortened names for convenience:
    //     * lf: left-front
    //     * rf: right-front
    //     * lb: left-back
    //     * rb: right-back

    // TODO: Fill in the actual TalonFX IDs.
    private TalonFX lfSpeedMotor = new TalonFX(0);
    private TalonFX rfSpeedMotor = new TalonFX(1);
    private TalonFX lbSpeedMotor = new TalonFX(2);
    private TalonFX rbSpeedMotor = new TalonFX(3);

    private TalonFX lfDirectionMotor = new TalonFX(4);
    private TalonFX rfDirectionMotor = new TalonFX(5);
    private TalonFX lbDirectionMotor = new TalonFX(6);
    private TalonFX rbDirectionMotor = new TalonFX(7);

    private double wheelP = 0;
    private double wheelI = 0;
    private double wheelD = 0;

    private SwerveWheel lfWheel = new SwerveWheel(
        this.lfSpeedMotor, this.lfDirectionMotor, wheelP, wheelI, wheelD);
    private SwerveWheel rfWheel = new SwerveWheel(
        this.rfSpeedMotor, this.rfDirectionMotor, wheelP, wheelI, wheelD);
    private SwerveWheel lbWheel = new SwerveWheel(
        this.lbSpeedMotor, this.lbDirectionMotor, wheelP, wheelI, wheelD);
    private SwerveWheel rbWheel = new SwerveWheel(
        this.rbSpeedMotor, this.rbDirectionMotor, wheelP, wheelI, wheelD);

    // Helper arrays for accessing subsets of our wheels.
    private SwerveWheel[] swerveWheels = {lfWheel, rfWheel, lbWheel, rbWheel};
    private SwerveWheel[] frontWheels = {lfWheel, rfWheel};
    private SwerveWheel[] backWheels = {lbWheel, rbWheel};
    private SwerveWheel[] leftWheels = {lfWheel, lbWheel};
    private SwerveWheel[] rightWheels = {rfWheel, rbWheel};

    // TODO: Need to init encoders, and verify that the value they report
    // can be used for wheel angle.
    private void setTranslateAngle(double desiredAngle) {
        for(int i = 0; i < swerveWheels.length; i++) {
            swerveWheels[i].setDirection(desiredAngle);
        }
    }

    /*
     * Set the wheels to point in a circle.
     */
    private void setTurnInPlaceAngle() {
        lfWheel.setDirection(45);
        rfWheel.setDirection(45 + 90);
        lbWheel.setDirection(45 + (90 * 2));
        rbWheel.setDirection(45 + (90 * 3));
    }

    /*
     * Determine wheel angles when translating and turning at the same time.
     *
     *     direction: The direction we want to move in degrees.
     *     turnMagnitude: A value from [-1, 1] which expresses how sharply we want to
     *                    turn counter-clockwise or clockwise.
     */
    private void setTranslateTurnAngle(double direction, double turnMagnitude) {
        direction = Utils.normalizeAngle(direction);
        // Determine what our front and back wheels are, relative to the direction
        // we're going.
        SwerveWheel[] relativeFrontWheels;
        SwerveWheel[] relativeBackWheels;
        if ( -135 < direction && direction <= -45 ) {
            // Move the bot left.
            relativeFrontWheels = leftWheels;
            relativeBackWheels = rightWheels;
        } else if ( -45 < direction && direction <= 45 ) {
            // Move the bot forwards.
            relativeFrontWheels = frontWheels;
            relativeBackWheels = backWheels;
        } else if ( 45 < direction && direction <= 135 ) {
            // Move the bot right.
            relativeFrontWheels = rightWheels;
            relativeBackWheels = leftWheels;
        } else {
            // Move the bot backwards.
            relativeFrontWheels = backWheels;
            relativeBackWheels = frontWheels;
        }

        double turnAngleOffset = turnMagnitude * 45;
        for(int i = 0; i < 2; i++) {
            relativeFrontWheels[i].setDirection(direction + turnAngleOffset);
            relativeBackWheels[i].setDirection(direction - turnAngleOffset);
        }
    }

    private void setSpeed(double speed) {
        for(int i = 0; i < swerveWheels.length; i++) {
            swerveWheels[i].setSpeed(speed);
        }
    }

    public void setDrive(double direction, double translatePower, double turnMagnitude) {
        if(translatePower == 0 && turnMagnitude != 0) {
            setTurnInPlaceAngle();
            setSpeed(turnMagnitude);
        } else {
            setTranslateTurnAngle(direction, turnMagnitude);
            setSpeed(translatePower);
        }
    }

    public void periodic() {
        for(int i = 0; i < swerveWheels.length; i++) {
            swerveWheels[i].periodic();
        }
    }
}

public class Robot extends TimedRobot {
    XboxController controller = new XboxController(0);
    AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);
    SwerveDrive swerve = new SwerveDrive();

    public Robot() {}

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    private double stickToDirection(double stickX, double stickY) {
        double angleRadians = Math.atan2(stickY, stickX);
        double angleDegrees = Math.toDegrees(angleRadians);

        // "angleDegrees" represents an angle on the Cartesian plane, where 0 degrees
        // points from the origin to the postive X-axis, and the angle increases when
        // moving in the counter-clockwise direction (such that 90 degrees points
        // from the origin to the positive Y-axis).
        //
        // Our swerve drive code accepts angles where "North" represents 0 degrees,
        // "East" represents 90 degrees, "West" represents -90 degrees, and "South"
        // represents -180/180 degrees. The following line performs this conversion.
        return (Utils.normalizeAngle(angleDegrees) - 90) * -1;
    }

    @Override
    public void teleopPeriodic() {
        double direction = stickToDirection(
            controller.getLeftX(), controller.getLeftY());
        double wheelSpeed = controller.getLeftTriggerAxis();
        double turnMagnitude = controller.getRightX();
        swerve.setDrive(direction, wheelSpeed, turnMagnitude);
        swerve.periodic();
    }
}

/*
    Unused funcion headers
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
*/
