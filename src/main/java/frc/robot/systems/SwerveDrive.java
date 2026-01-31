// FRC Team 4034

package frc.robot.systems;

import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;

public class SwerveDrive {
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
