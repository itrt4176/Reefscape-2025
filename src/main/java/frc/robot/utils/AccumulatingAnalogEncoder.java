package frc.robot.utils;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class AccumulatingAnalogEncoder extends AnalogEncoder {

  private MedianFilter filter;

  private int wrapCount;

  private final static double ENDS_RANGE = 20 / 360.0; 

  public AccumulatingAnalogEncoder(int channel, int windowSize) {
    super(channel);
    filter = new MedianFilter(windowSize);
    filter.calculate(super.get());
    wrapCount = 0;
  }

  @Override
  public double get() {
      double last = filter.lastValue();
      double current = filter.calculate(super.get());

      if (inRange(current, 0.0, ENDS_RANGE) && inRange(last, 1.0 - ENDS_RANGE, 1.0)) {
        wrapCount--;
      } else if (inRange(current, 1.0 - ENDS_RANGE, 1.0) && inRange(last, 0.0, ENDS_RANGE)) {
        wrapCount++;
      }

      return current + wrapCount;
  }

  public double getRaw() {
    return super.get();
  }

  private final static boolean inRange(double value, double min, double max) {
    return value >= min && value <= max;
  }
}
