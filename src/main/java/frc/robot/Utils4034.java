// FRC Team 4034

package frc.robot;

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
