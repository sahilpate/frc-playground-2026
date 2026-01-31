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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.ModuleConsts;

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

class SwerveDrive {
    // Shortened names for convenience:
    //     * lf: left-front
    //     * rf: right-front
    //     * lb: left-back
    //     * rb: right-back
    private final SwerveModule lfModule = new SwerveModule(
        ConfigConsts.lfSpeedMotorId,
        ConfigConsts.lfDirectionMotorId,
        ConfigConsts.lfEncoderId,
        ConfigConsts.reverseLfSpeedMotor,
        ConfigConsts.reverseLfDirectionMotor,
        ConfigConsts.reverseLfEncoder);

    private final SwerveModule rfModule = new SwerveModule(
        ConfigConsts.rfSpeedMotorId,
        ConfigConsts.rfDirectionMotorId,
        ConfigConsts.rfEncoderId,
        ConfigConsts.reverseRfSpeedMotor,
        ConfigConsts.reverseRfDirectionMotor,
        ConfigConsts.reverseRfEncoder);

    private final SwerveModule lbModule = new SwerveModule(
        ConfigConsts.lbSpeedMotorId,
        ConfigConsts.lbDirectionMotorId,
        ConfigConsts.lbEncoderId,
        ConfigConsts.reverseLbSpeedMotor,
        ConfigConsts.reverseLbDirectionMotor,
        ConfigConsts.reverseLbEncoder);

    private final SwerveModule rbModule = new SwerveModule(
        ConfigConsts.rbSpeedMotorId,
        ConfigConsts.rbDirectionMotorId,
        ConfigConsts.rbEncoderId,
        ConfigConsts.reverseRbSpeedMotor,
        ConfigConsts.reverseRbDirectionMotor,
        ConfigConsts.reverseRbEncoder);

    private AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);

    public SwerveDrive() {
        // Calibrate the the NavXMXP in a separate thread, so that it doesn't block
        // other initialization.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                navxMxp.reset();
            } catch (Exception e) {
            }
        }).start();
    }

    public void setModules(double xSpeed, double ySpeed, double turnSpeed) {
        // TODO: Check that "getRotation2d()" returns an angle in radians, and that
        // it is CCW positive.
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, turnSpeed, navxMxp.getRotation2d());

        SwerveModuleState[] moduleStates =
            DriveConsts.driveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates, DriveConsts.maxMetersPerSecToMotorSpeed);

        lfModule.setDesiredState(moduleStates[0]);
        rfModule.setDesiredState(moduleStates[1]);
        lbModule.setDesiredState(moduleStates[2]);
        rbModule.setDesiredState(moduleStates[3]);
    }

    public void stopModules() {
        lfModule.stop();
        rfModule.stop();
        lbModule.stop();
        rbModule.stop();
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
    public void autonomousPeriodic() {
        swerve.stopModules();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        swerve.stopModules();
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
