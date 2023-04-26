package org.cohere.od.generation;

import java.util.ArrayList;
import java.util.List;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.random.CorrelatedRandomVectorGenerator;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.RandomDataGenerator;
import org.hipparchus.random.RandomGenerator;
import org.orekit.estimation.measurements.AngularRaDec;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.estimation.measurements.generation.AngularRaDecBuilder;
import org.orekit.estimation.measurements.generation.ContinuousScheduler;
import org.orekit.estimation.measurements.generation.Generator;
import org.orekit.estimation.measurements.generation.MeasurementBuilder;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.Propagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DatesSelector;
import org.orekit.time.FixedStepSelector;
import org.orekit.time.TimeScalesFactory;

/**
 * Helper class for generating measurements.
 */
public class MeasurementGenerator {

  private MeasurementGenerator() {
  }

  /**
   * {@code randomSeed} defaults to -1 (no noise).
   *
   * @see MeasurementGenerator#generateRaDecMeasurements(Propagator, GroundStation, double[],
   * AbsoluteDate, AbsoluteDate, double, int)
   */
  public static List<ObservedMeasurement<?>> generateRaDecMeasurements(Propagator propagator,
      GroundStation groundStation, double[] raDecSigmas, AbsoluteDate startTime,
      AbsoluteDate stopTime, double step) {
    return generateRaDecMeasurements(propagator, groundStation, raDecSigmas, startTime, stopTime,
        step, -1);
  }

  /**
   * Generate a list of RA/Dec measurements.
   *
   * @param propagator    The spacecraft propagator to use.
   * @param groundStation The ground station observing the spacecraft.
   * @param raDecSigmas   The standard deviations of right ascension and declination, respectively.
   * @param startTime     The start time of the measurement set.
   * @param stopTime      The stop time of the measurement set.
   * @param step          The time step between measurements.
   * @param randomSeed    The random seed for generating noise on the measurements. Use -1 for no
   *                      noise.
   * @return A list of RA/Dec {@link ObservedMeasurement}s.
   */
  public static List<ObservedMeasurement<?>> generateRaDecMeasurements(Propagator propagator,
      GroundStation groundStation, double[] raDecSigmas, AbsoluteDate startTime,
      AbsoluteDate stopTime, double step, int randomSeed) {

    Generator generator = new Generator();
    ObservableSatellite satellite = generator.addPropagator(propagator);

    RandomDataGenerator random = randomSeed == -1 ? null : new RandomDataGenerator(randomSeed);
    double[] raDecWeights = new double[]{1.0, 1.0};
    MeasurementBuilder<?> measurementBuilder = getRaDecBuilder(random, groundStation, satellite,
        raDecSigmas, raDecWeights);

    DatesSelector selector = new FixedStepSelector(step, TimeScalesFactory.getUTC());
    generator.addScheduler(new ContinuousScheduler<>(measurementBuilder, selector));
    return new ArrayList<>(generator.generate(startTime, stopTime));

  }

  private static MeasurementBuilder<AngularRaDec> getRaDecBuilder(RandomGenerator random,
      GroundStation groundStation, ObservableSatellite satellite, double[] sigmas,
      double[] weights) {

    double raSigma = sigmas[0];
    double decSigma = sigmas[1];

    RealMatrix covariance = MatrixUtils.createRealDiagonalMatrix(
        new double[]{raSigma * raSigma, decSigma * decSigma});

    CorrelatedRandomVectorGenerator noiseGenerator = random == null ? null
        : new CorrelatedRandomVectorGenerator(covariance, 1e-10,
            new GaussianRandomGenerator(random));

    return new AngularRaDecBuilder(noiseGenerator, groundStation, FramesFactory.getGCRF(),
        new double[]{raSigma, decSigma}, weights, satellite);
  }

}
