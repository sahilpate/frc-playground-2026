// FRC Team 4034

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ConfigConst {
        // Shortened names for convenience:
        //     * lf: left-front
        //     * rf: right-front
        //     * lb: left-back
        //     * rb: right-back

        // TODO: Fill in the actual IDs and booleans.
        public static final int lfSpeedMotorId = 0;
        public static final int lfDirectionMotorId = 1;
        public static final int lfEncoderId = 2;
        public static final boolean reverseLfSpeedMotor = false;
        public static final boolean reverseLfDirectionMotor = false;
        public static final boolean reverseLfEncoder = false;

        public static final int rfSpeedMotorId = 0;
        public static final int rfDirectionMotorId = 1;
        public static final int rfEncoderId = 2;
        public static final boolean reverseRfSpeedMotor = false;
        public static final boolean reverseRfDirectionMotor = false;
        public static final boolean reverseRfEncoder = false;

        public static final int lbSpeedMotorId = 0;
        public static final int lbDirectionMotorId = 1;
        public static final int lbEncoderId = 2;
        public static final boolean reverseLbSpeedMotor = false;
        public static final boolean reverseLbDirectionMotor = false;
        public static final boolean reverseLbEncoder = false;

        public static final int rbSpeedMotorId = 0;
        public static final int rbDirectionMotorId = 1;
        public static final int rbEncoderId = 2;
        public static final boolean reverseRbSpeedMotor = false;
        public static final boolean reverseRbDirectionMotor = false;
        public static final boolean reverseRbEncoder = false;
    }

    public static final class ModuleConstants {
        /******************* Constants for direction wheels *******************/
        // TODO: Is the wheel diameter actually 4 inches?
        public static final double wheelDiameterMeters = Units.inchesToMeters(4);
        // TODO: I just picked 6.82 arbitrarily from WCP's gear ratio docs. We need
        // to figure out what value actually belongs here. See:
        // docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options
        public static final double wheelRotationPerSpeedMotorRotation = 1 / 6.82;
        // A.K.A., the circumference of a wheel.
        public static final double metersPerWheelRotation = Math.PI * wheelDiameterMeters;
        public static final double speedMotorRotationToMeters =
            wheelRotationPerSpeedMotorRotation * metersPerWheelRotation;

        /********************* Constants for angle wheels *********************/
        // TODO: WCP docs say that this is the conversion in all configurations.
        // Validate that this is actually true?
        // docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options
        public static final double directionMotorRotationPerWheelRotation = 12.1;
        public static final double wheelRotationPerDirectionMotorRotation =
            1 / directionMotorRotationPerWheelRotation;
        public static final double radiansPerWheelRotation = 2 * Math.PI;
        public static final double radiansPerDirectionMotorRotation =
            wheelRotationPerDirectionMotorRotation * radiansPerWheelRotation;
        public static final double motorRotationsPerRadian =
            (1 / radiansPerWheelRotation) * directionMotorRotationPerWheelRotation;

        // "P" component of our direction PID controller.
        public static final double directionP = 0;

        // The factor we divide our desired "metersPerSec" by to actually set our
        // motor's speed.
        // TODO: This value's been copy+pasted from the reference project. Once we've
        // validated that our `getDrivePosition()` works, we can hopefully use
        // `getDriveVelocity()` to map our [-1, 1] motor inputs to the max velocity
        // we want the wheels to move at.
        public static final double maxMetersPerSecToMotorSpeed = 5;
    }
}
