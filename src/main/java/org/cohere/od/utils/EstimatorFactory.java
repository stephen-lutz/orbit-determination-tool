package org.cohere.od.utils;

import java.util.Comparator;
import java.util.List;
import org.cohere.od.observer.OrbitDeterminationObserver;
import org.hipparchus.linear.QRDecomposer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.GaussNewtonOptimizer;
import org.orekit.estimation.leastsquares.BatchLSEstimator;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.conversion.OrbitDeterminationPropagatorBuilder;
import org.orekit.propagation.conversion.PropagatorBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeStamped;

public class EstimatorFactory {

  private static final double CONVERGENCE_THRESHOLD = 1e-3;
  private static final int MAX_ITERATIONS = 25;
  private static final double SINGULARITY_THRESHOLD = 1e-11;


  private EstimatorFactory() {
  }

  /**
   * Creates a batch LS estimator to perform an OD. The OD epoch is the epoch of the final
   * measurement.
   *
   * @param propagatorBuilder The propagator builder to use.
   * @param measurements      The list of measurements to use.
   * @return The configured {@link BatchLSEstimator} object to use.
   */
  public static BatchLSEstimator createBatchLsEstimator(
      OrbitDeterminationPropagatorBuilder propagatorBuilder,
      List<ObservedMeasurement<?>> measurements) {

    // Shift the propagator builder to the epoch of the last measurement.
    measurements.sort(Comparator.comparing(TimeStamped::getDate));
    AbsoluteDate odEpoch = measurements.get(measurements.size() - 1).getDate();
    shiftToOdEpoch(propagatorBuilder, odEpoch);

    GaussNewtonOptimizer optimizer = new GaussNewtonOptimizer(
        new QRDecomposer(SINGULARITY_THRESHOLD), false);
    BatchLSEstimator estimator = new BatchLSEstimator(optimizer, propagatorBuilder);

    estimator.setParametersConvergenceThreshold(CONVERGENCE_THRESHOLD);
    estimator.setMaxIterations(MAX_ITERATIONS);
    estimator.setMaxEvaluations(MAX_ITERATIONS);

    for (ObservedMeasurement<?> measurement : measurements) {
      estimator.addMeasurement(measurement);
    }

    // Note: the observer is used for obtaining results for each evaluation of the estimator.
    Orbit initialOrbit = propagatorBuilder.buildPropagator(
        propagatorBuilder.getSelectedNormalizedParameters()).getInitialState().getOrbit();
    estimator.setObserver(new OrbitDeterminationObserver(initialOrbit, estimator));

    return estimator;
  }

  private static void shiftToOdEpoch(OrbitDeterminationPropagatorBuilder builder, AbsoluteDate odEpoch) {
    Propagator propagator = builder.buildPropagator(
        builder.getSelectedNormalizedParameters());
    if (!propagator.getInitialState().getDate().isEqualTo(odEpoch)) {
      SpacecraftState propagatedState = propagator.propagate(odEpoch);
      builder.resetOrbit(propagatedState.getOrbit());
    }
  }

}
