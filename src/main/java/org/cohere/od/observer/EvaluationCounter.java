package org.cohere.od.observer;

import org.orekit.estimation.measurements.EstimatedMeasurement;
import org.orekit.estimation.measurements.ObservedMeasurement;

/**
 * Evaluation counting.
 * <p>
 * Based on Orekit tutorial EvaluationCounter class.
 */
class EvaluationCounter<T extends ObservedMeasurement<T>> {

  /**
   * Number of active (i.e. positive weight) measurements.
   */
  private int active;
  /**
   * Total number of measurements.
   */
  private int total;

  /**
   * Add a measurement evaluation.
   *
   * @param evaluation measurement evaluation to add
   */
  public void add(EstimatedMeasurement<T> evaluation) {
    ++total;
    if (evaluation.getStatus() == EstimatedMeasurement.Status.PROCESSED) {
      ++active;
    }
  }

  /**
   * Format an active/total count.
   *
   * @param size field minimum size
   * @return formatted count
   */
  public String format(int size) {
    final StringBuilder builder = new StringBuilder();
    builder.append(active);
    builder.append('/');
    builder.append(total);
    while (builder.length() < size) {
      if (builder.length() % 2 == 0) {
        builder.insert(0, ' ');
      } else {
        builder.append(' ');
      }
    }
    return builder.toString();
  }

}
