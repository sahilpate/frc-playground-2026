// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

class SwerveModule {
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

        this.directionController = new PIDController(ModuleConstants.directionP, 0, 0);
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
            motorRotations * ModuleConstants.speedMotorRotationToMeters;

        int reverseFactor = (this.reverseSpeedMotor ? -1 : 1);
        return meters * reverseFactor;
    }

    public double getDriveVelocity() {
        double motorRotationsPerSec =
            this.speedMotor.getVelocity().getValue().baseUnitMagnitude();

        // Do the following unit conversion:
        //     (MotorRotation / Sec) * (Meters / MotorRotation) -> (Meters / Sec)
        double metersPerSec =
            motorRotationsPerSec * ModuleConstants.speedMotorRotationToMeters;

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
            motorRotations * ModuleConstants.radiansPerDirectionMotorRotation;

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
            motorRotationsPerSec * ModuleConstants.radiansPerDirectionMotorRotation;

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
            absoluteEncoderRadians * ModuleConstants.motorRotationsPerRadian;
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
            state.speedMetersPerSecond / ModuleConstants.maxMetersPerSecToMotorSpeed;
        speedMotor.set(newSpeedMotorVal);

        double newDirectionMotorVal = directionController.calculate(
            getTurningPosition(), state.angle.getRadians());
        directionMotor.set(newDirectionMotorVal);
    }

    public void stop() {
        speedMotor.set(0);
        directionMotor.set(0);
    }
}

class SwerveDrive {
    // Shortened names for convenience:
    //     * lf: left-front
    //     * rf: right-front
    //     * lb: left-back
    //     * rb: right-back
}

public class Robot extends TimedRobot {
    XboxController controller = new XboxController(0);
    AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);

    public Robot() {}

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    /*
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
    */

    @Override
    public void teleopPeriodic() {}
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
