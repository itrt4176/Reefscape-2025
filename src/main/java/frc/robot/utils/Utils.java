package frc.robot.utils;

public final class Utils {
    /**
     * Re-maps a number from one range to another. That is, a value of `inputLow` would
     * get mapped to `outputLow`, a value of `inputMin` to `outputHigh`, values
     * in-between to values in-between, etc.
     * 
     * @param input the number to map
     * @param inputMin the lower bound of the value's current range
     * @param inputMax the upper bound of the value's current range
     * @param outputMin the lower bound of the value's target range
     * @param outputMax the upper bound of the value's target range
     * @return the mapped value
     */
    public static final int mapRange(
        int input,
        int inputMin,
        int inputMax,
        int outputMin,
        int outputMax
    ) {
        return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
    }

    /**
     * Re-maps a number from one range to another. That is, a value of `inputLow` would
     * get mapped to `outputLow`, a value of `inputMin` to `outputHigh`, values
     * in-between to values in-between, etc.
     * 
     * @param input the number to map
     * @param inputMin the lower bound of the value's current range
     * @param inputMax the upper bound of the value's current range
     * @param outputMin the lower bound of the value's target range
     * @param outputMax the upper bound of the value's target range
     * @return the mapped value
     */
    public static final double mapRange(
        double input,
        double inputMin,
        double inputMax,
        double outputMin,
        double outputMax
    ) {
        return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
    }
}
