// FRC Team 4034

package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.ModuleConsts;

public class SwerveModule {
    private final TalonFX speedMotor;
    private final TalonFX directionMotor;
    // Tracks the current angle of motor. This retains its values over a powercycle,
    // unlike the TalonFX's builtin encoder.
    private final CoreCANcoder absoluteEncoder;

    private final boolean reverseSpeedMotor;
    private final boolean reverseDirectionMotor;
    private final boolean reverseAbsoluteEncoder;

    private final PIDController directionController;

    public SwerveModule(int speedMotorId, int directionMotorId, int absoluteEncoderId,
                        boolean reverseSpeedMotor, boolean reverseDirectionMotor,
                        boolean reverseAbsoluteEncoder) {
        this.speedMotor = new TalonFX(speedMotorId);
        this.directionMotor = new TalonFX(directionMotorId);
        this.absoluteEncoder = new CoreCANcoder(absoluteEncoderId);

        this.reverseSpeedMotor = reverseSpeedMotor;
        this.reverseDirectionMotor = reverseDirectionMotor;
        this.reverseAbsoluteEncoder = reverseAbsoluteEncoder;

        this.directionController = new PIDController(ModuleConsts.directionP, 0, 0);
        this.directionController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        // TODO: Validate that the value this reports is accurate. If so, then
        // getDriveVelocity() should also hopefully be accurate.
        double motorRotations =
            this.speedMotor.getPosition().getValue().baseUnitMagnitude();

        // Do the following unit conversion:
        //     (MotorRotation) * (Meters / MotorRotation) -> (Meters)
        double meters =
            motorRotations * ModuleConsts.speedMotorRotationToMeters;

        int reverseFactor = (this.reverseSpeedMotor ? -1 : 1);
        return meters * reverseFactor;
    }

    public double getDriveVelocity() {
        double motorRotationsPerSec =
            this.speedMotor.getVelocity().getValue().baseUnitMagnitude();

        // Do the following unit conversion:
        //     (MotorRotation / Sec) * (Meters / MotorRotation) -> (Meters / Sec)
        double metersPerSec =
            motorRotationsPerSec * ModuleConsts.speedMotorRotationToMeters;

        int reverseFactor = (this.reverseSpeedMotor ? -1 : 1);
        return metersPerSec * reverseFactor;
    }

    public double getTurningPosition() {
        // TODO: Validate that the value this reports is accurate.
        double motorRotations =
            this.directionMotor.getPosition().getValue().baseUnitMagnitude();

        // Do the following unit coversion:
        //     (MotorRotation) * (WheelRotationRadians / MotorRotation)
        //         -> WheelRotationRadians
        double radians =
            motorRotations * ModuleConsts.radiansPerDirectionMotorRotation;

        int reverseFactor = (this.reverseDirectionMotor ? -1 : 1);
        return radians * reverseFactor;
    }

    public double getTurningVelocity() {
        double motorRotationsPerSec =
            this.directionMotor.getVelocity().getValue().baseUnitMagnitude();

        // Do the following unit coversion:
        //     (MotorRotation / Sec ) * (WheelRotationRadians / MotorRotation)
        //         -> WheelRotationRadians
        double radiansPerSec =
            motorRotationsPerSec * ModuleConsts.radiansPerDirectionMotorRotation;

        int reverseFactor = (this.reverseDirectionMotor ? -1 : 1);
        return radiansPerSec * reverseFactor;
    }

    public double getAbsoluteEncoderRad() {
        double angleRad =
            this.absoluteEncoder.getAbsolutePosition().getValue().baseUnitMagnitude();
        return (this.reverseAbsoluteEncoder ? -angleRad : angleRad);
    }

    public void resetEncoders() {
        this.speedMotor.setPosition(0);

        double absoluteEncoderRadians = getAbsoluteEncoderRad();
        // Do the following unit coversion:
        //     (Radians) * (MotorRotation / Radians) -> MotorRotation
        double currMotorRotations =
            absoluteEncoderRadians * ModuleConsts.motorRotationsPerRadian;
        this.directionMotor.setPosition(currMotorRotations);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // This "if" condition prevents the wheels from swinging back to their
        // neutral position when the joysticks are let go.
        // TODO: The "0.001" may need to be increased depending on the behavior we
        // observe when testing this.
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state.optimize(getState().angle);

        double newSpeedMotorVal =
            state.speedMetersPerSecond / DriveConsts.maxMetersPerSecToMotorSpeed;
        speedMotor.set(newSpeedMotorVal);

        double newDirectionMotorVal = directionController.calculate(
            getTurningPosition(), state.angle.getRadians());
        directionMotor.set(newDirectionMotorVal);
    }

    public void stop() {
        speedMotor.set(0);
        directionMotor.set(0);
    }

    /***********************************************************************************/
    /*                      Helper functions/variables for debugging                   */
    /***********************************************************************************/
    // Make the wheels rotate once for speed.
    private double nextOneSpeedRotationMeters = 0;
    public void setNextOneSpeedRotation(boolean backwards) {
        double currDrivePositionMeters = getDrivePosition();
        if(backwards) {
            nextOneSpeedRotationMeters =
                currDrivePositionMeters - ModuleConsts.metersPerWheelRotation;
        } else {
            nextOneSpeedRotationMeters =
                currDrivePositionMeters + ModuleConsts.metersPerWheelRotation;
        }
    }
    public void moveForOneSpeedRotation() {
        double currDrivePositionMeters = getDrivePosition();
        // 5cm tolerance?
        double toleranceMeters = 0.05;
        if(currDrivePositionMeters < (nextOneSpeedRotationMeters - toleranceMeters)) {
            speedMotor.set(0.1);
        } else if (currDrivePositionMeters > (nextOneSpeedRotationMeters + toleranceMeters)) {
            speedMotor.set(-0.1);
        }
    }

    // Make the wheels rotate once for direction.
    private double nextOneDirectionRotationRad = 0;
    public void setNextOneDirectionRotation(boolean backwards) {
        double currDriveDirectionRad = getTurningPosition();
        if(backwards) {
             nextOneDirectionRotationRad =
                 currDriveDirectionRad - ModuleConsts.radiansPerWheelRotation;
        } else {
            nextOneDirectionRotationRad =
                 currDriveDirectionRad + ModuleConsts.radiansPerWheelRotation;
        }
    }
    public void moveForOneDirectionRotation() {
        double currDriveDirectionRad = getTurningPosition();
        // 5 degrees tolerance?
        double toleranceRad = 5 * (Math.PI / 180);
        if(currDriveDirectionRad < (nextOneDirectionRotationRad - toleranceRad)) {
            directionMotor.set(0.1);
        } else if (currDriveDirectionRad > (nextOneDirectionRotationRad + toleranceRad)) {
            directionMotor.set(-0.1);
        }
    }

    // TODO (for Sahil): Write a function that makes the wheels spin (speed or direction)
    // at some configurable speed for X seconds. Mainly for deciding what we want our
    // constants to be.
}
