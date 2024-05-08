package frc.robot.Utils;

import edu.wpi.first.math.util.Units;

public class Helpers {
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
    /** Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0. */
    public static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }
        /**
     * Wraps the angle so that its always between 0 -> (2 * pi)
     *
     * @param angle that needs to be wrapped
     * @param radians boolean to calculate radians vs degrees
     */
    public static double angleWrap(double angle, boolean radians) {
        angle = radians ? angle : Units.degreesToRadians(angle);
        angle = angle % (2.0 * Math.PI);
        if (angle < 0) {
            angle += (2.0 * Math.PI);
        }
        return radians ? angle : Units.radiansToDegrees(angle);
    }

    public static double joystickToAngle(double x, double y) {
        return Units.radiansToDegrees(Math.atan2(y, x));
    }
    public static Vector2d axisToSegmentedUnitCircleRadians(double x, double y, int[] array) {
        double angle = joystickToAngle(x, y);
        angle = (angle + 360) % 360;
        angle = angle / 360;
        int segment = (int) (angle * array.length);
        double power = Math.hypot(x, y);
        if (power > 1) {
            power = 1;
        }
        return new Vector2d(
                power * Math.cos(Units.degreesToRadians(array[segment])),
                power * Math.sin(Units.degreesToRadians(array[segment])));
    }












}
