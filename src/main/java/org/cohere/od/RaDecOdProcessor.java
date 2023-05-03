package org.cohere.od;

import java.util.List;
import lombok.NonNull;
import lombok.extern.log4j.Log4j2;
import org.cohere.od.models.StateAndCovariance;
import org.hipparchus.linear.RealMatrix;
import org.orekit.errors.OrekitException;
import org.orekit.estimation.leastsquares.BatchLSEstimator;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;

/**
 * An {@link OdProcessor} to process right ascension and declination measurements using a batch
 * least-squares estimator.
 */
@Log4j2
public class RaDecOdProcessor implements OdProcessor {

  private final BatchLSEstimator estimator;

  public RaDecOdProcessor(BatchLSEstimator estimator) {
    this.estimator = estimator;
  }

  /**
   * Given the initial state and set of measurements, perform a batch least-squares orbit
   * determination (OD) to obtain a state and covariance at the epoch of the final measurement.
   *
   * @param initialState The initial state of the spacecraft.
   * @param measurements The set of measurements to use.
   * @return The estimate state and covariance of the spacecraft at the epoch of the final
   * measurement.
   */
  @Override
  public StateAndCovariance processMeasurements(@NonNull SpacecraftState initialState,
      @NonNull List<ObservedMeasurement<?>> measurements) {

    if (measurements.isEmpty()) {
      throw new IllegalArgumentException("Must provide at least 1 measurement.");
    }

    // Execute the OD.
    Propagator estimatedPropagator;
    try {
      estimatedPropagator = estimator.estimate()[0];
    } catch (OrekitException ex) {
      throw new IllegalStateException("Failed to execute OD: " + ex.getMessage());
    }

    SpacecraftState estimatedState = estimatedPropagator.getInitialState();
    RealMatrix finalCovarianceMatrix = estimator.getPhysicalCovariances(Double.MIN_VALUE)
        .getSubMatrix(0, 5, 0, 5);
    StateCovariance estimatedCovariance = new StateCovariance(finalCovarianceMatrix,
        estimatedState.getDate(), estimatedState.getFrame(), OrbitType.CARTESIAN,
        PositionAngle.MEAN);

    return new StateAndCovariance(estimatedCovariance, estimatedState);
  }

}
