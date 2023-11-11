package frc.utils;

public class OperatorUtils {
    public static int getSign(double value) {
        if (value > 0)
            return 1;
        if (value < 0)
            return -1;
        else
            return 0;
    }

    public static double continous_deadband(double value, double deadband) {
        double absValue = Math.abs(value);
        if (absValue < deadband)
            return 0;
        return getSign(value) * (absValue - deadband);
    }

    public static double clampValue(double value, double limit) {
        double absValue = Math.abs(value);
        if (absValue < limit)
            return value;
        return getSign(value) * (limit);
    }


}
