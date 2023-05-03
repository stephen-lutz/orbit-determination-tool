package org.cohere.od.generation;

import java.util.ArrayList;
import java.util.List;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.random.CorrelatedRandomVectorGenerator;
import org.hipparchus.random.GaussianRandomGenerator;
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

  private final Propagator propagator;
  private final RandomGenerator random;

  /**
   * Creates a new measurement generator.
   *
   * @param propagator The propagator corresponding to the spacecraft for which measurements will be
   *                   generated.
   * @param random     The random number generator used to add Gaussian noise to the measurements.
   */
  public MeasurementGenerator(Propagator propagator, RandomGenerator random) {
    this.propagator = propagator;
    this.random = random;
  }

  /**
   * Constructor with {@code random} defaulted to null (no noise on measurements).
   *
   * @see MeasurementGenerator#MeasurementGenerator(Propagator, RandomGenerator)
   */
  public MeasurementGenerator(Propagator propagator) {
    this(propagator, null);
  }

  /**
   * Generate a list of RA/Dec measurements.
   *
   * @param groundStation The ground station observing the spacecraft.
   * @param sigmas        The standard deviations of right ascension and declination, respectively.
   * @param startTime     The start time of the measurement set.
   * @param stopTime      The stop time of the measurement set.
   * @param step          The time step between measurements.
   * @return A list of RA/Dec {@link ObservedMeasurement}s.
   */
  public List<ObservedMeasurement<?>> generateRaDecMeasurements(GroundStation groundStation,
      double[] sigmas, double[] weights, AbsoluteDate startTime, AbsoluteDate stopTime,
      double step) {

    Generator generator = new Generator();
    ObservableSatellite satellite = generator.addPropagator(propagator);

    MeasurementBuilder<?> measurementBuilder = getRaDecBuilder(groundStation, satellite, sigmas,
        weights);
    DatesSelector selector = new FixedStepSelector(step, TimeScalesFactory.getUTC());

    generator.addScheduler(new ContinuousScheduler<>(measurementBuilder, selector));
    return new ArrayList<>(generator.generate(startTime, stopTime));
  }

  private MeasurementBuilder<AngularRaDec> getRaDecBuilder(GroundStation groundStation,
      ObservableSatellite satellite, double[] sigmas, double[] weights) {

    // Add Gaussian noise to measurements.
    CorrelatedRandomVectorGenerator noiseGenerator = null;
    if (random != null) {

      final double SMALL = 1e-15;
      double raSigma = sigmas[0];
      double decSigma = sigmas[1];

      RealMatrix covariance = MatrixUtils.createRealDiagonalMatrix(
          new double[]{raSigma * raSigma, decSigma * decSigma});

      noiseGenerator = new CorrelatedRandomVectorGenerator(covariance, SMALL,
          new GaussianRandomGenerator(random));
    }

    return new AngularRaDecBuilder(noiseGenerator, groundStation, FramesFactory.getGCRF(),
        sigmas, weights, satellite);
  }

}
