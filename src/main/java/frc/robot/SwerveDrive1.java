// FRC Team 4034
//
// Sahil's first cut at a swerve implementation. It might be good enough, but on
// second thought, I'd rather utilize the functionality of WPILib to avoid applying
// some hacky workarounds to make this implementation drive cleanly.
//
// This implementation is mainly inspired from:
// https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

class Utils {
    // Represent the inputted angle as an angle in the [-180, 180) degree range.
    public static double normalizeAngle(double angle) {
        double normalizedAngle = angle % 360;
        if (normalizedAngle < 0) {
            normalizedAngle += 360;
        }
        // normalizedAngle is currently in the [0, 360) range. Subtract 180 to put it
        // into the [-180, 180) range.
        return normalizedAngle - 180;
    }
}

class SwerveWheel1 {
    TalonFX speedMotor;
    double currSpeed;

    TalonFX dirMotor;
    PIDController dirController;

    public SwerveWheel1(TalonFX speedMotor, TalonFX dirMotor,
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

        // TODO_OLD: Need to apply some conversion factor to this to get an angle in degrees.

        // TODO_OLD: Need to use encoder to offset this so that the angle we report is the
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

class SwerveDrive1 {
    // Shortened names for convenience:
    //     * lf: left-front
    //     * rf: right-front
    //     * lb: left-back
    //     * rb: right-back

    // TODO_OLD: Fill in the actual TalonFX IDs.
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

    private SwerveWheel1 lfWheel = new SwerveWheel1(
        this.lfSpeedMotor, this.lfDirectionMotor, wheelP, wheelI, wheelD);
    private SwerveWheel1 rfWheel = new SwerveWheel1(
        this.rfSpeedMotor, this.rfDirectionMotor, wheelP, wheelI, wheelD);
    private SwerveWheel1 lbWheel = new SwerveWheel1(
        this.lbSpeedMotor, this.lbDirectionMotor, wheelP, wheelI, wheelD);
    private SwerveWheel1 rbWheel = new SwerveWheel1(
        this.rbSpeedMotor, this.rbDirectionMotor, wheelP, wheelI, wheelD);

    // Helper arrays for accessing subsets of our wheels.
    private SwerveWheel1[] swerveWheels = {lfWheel, rfWheel, lbWheel, rbWheel};
    private SwerveWheel1[] frontWheels = {lfWheel, rfWheel};
    private SwerveWheel1[] backWheels = {lbWheel, rbWheel};
    private SwerveWheel1[] leftWheels = {lfWheel, lbWheel};
    private SwerveWheel1[] rightWheels = {rfWheel, rbWheel};

    // TODO_OLD: Need to init encoders, and verify that the value they report
    // can be used for wheel angle.
    // private void setTranslateAngle(double desiredAngle) {
    //     for(int i = 0; i < swerveWheels.length; i++) {
    //         swerveWheels[i].setDirection(desiredAngle);
    //     }
    // }

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
        SwerveWheel1[] relativeFrontWheels;
        SwerveWheel1[] relativeBackWheels;
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
