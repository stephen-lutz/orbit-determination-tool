package org.cohere.od;

import java.util.Comparator;
import java.util.List;
import lombok.NonNull;
import lombok.extern.log4j.Log4j2;
import org.cohere.od.observer.OrbitDeterminationObserver;
import org.cohere.od.utils.AstroUtils;
import org.hipparchus.linear.QRDecomposer;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.optim.nonlinear.vector.leastsquares.GaussNewtonOptimizer;
import org.orekit.attitudes.InertialProvider;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.errors.OrekitException;
import org.orekit.estimation.leastsquares.BatchLSEstimator;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.forces.ForceModel;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.forces.radiation.IsotropicRadiationSingleCoefficient;
import org.orekit.forces.radiation.SolarRadiationPressure;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.conversion.DormandPrince853IntegratorBuilder;
import org.orekit.propagation.conversion.NumericalPropagatorBuilder;
import org.orekit.propagation.conversion.ODEIntegratorBuilder;
import org.orekit.propagation.conversion.OrbitDeterminationPropagatorBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeStamped;
import org.orekit.utils.Constants;

/**
 * Library to perform an orbit determination.
 * <p>
 * Uses a batch least-squares approach.
 */
@Log4j2
public class OdProcessor {

  private OdProcessor() {
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
  public static StateAndCovariance processMeasurements(@NonNull SpacecraftState initialState,
      @NonNull List<ObservedMeasurement<?>> measurements) {

    if (measurements.isEmpty()) {
      throw new IllegalArgumentException("Must provide at least 1 measurement.");
    }

    // Sort the measurements.
    measurements.sort(Comparator.comparing(TimeStamped::getDate));
    AbsoluteDate odEpoch = measurements.get(measurements.size() - 1).getDate();

    // Create the estimator.
    OrbitDeterminationPropagatorBuilder propagatorBuilder = createPropagatorBuilder(initialState,
        odEpoch);
    BatchLSEstimator estimator = createBatchEstimator(propagatorBuilder, initialState.getOrbit(),
        measurements);

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

  /**
   * Creates the batch LS estimator to perform the OD.
   *
   * @param propagatorBuilder The propagator builder to use.
   * @param initialOrbit      The initial orbit.
   * @param measurements      The list of measurements to use.
   * @return The configured {@link BatchLSEstimator} object to use.
   */
  static BatchLSEstimator createBatchEstimator(
      OrbitDeterminationPropagatorBuilder propagatorBuilder, Orbit initialOrbit,
      List<ObservedMeasurement<?>> measurements) {

    // TODO: make these configurable
    GaussNewtonOptimizer optimizer = new GaussNewtonOptimizer(new QRDecomposer(1e-11), false);
    BatchLSEstimator estimator = new BatchLSEstimator(optimizer, propagatorBuilder);

    estimator.setParametersConvergenceThreshold(1e-3);
    estimator.setMaxIterations(20);
    estimator.setMaxEvaluations(25);

    for (ObservedMeasurement<?> measurement : measurements) {
      estimator.addMeasurement(measurement);
    }

    // Note: the observer is used for obtaining results for each evaluation of the estimator.
    estimator.setObserver(new OrbitDeterminationObserver(initialOrbit, estimator));

    return estimator;
  }

  static NumericalPropagatorBuilder createPropagatorBuilder(SpacecraftState initialState) {
    return createPropagatorBuilder(initialState, initialState.getDate());
  }

  /**
   * Creates a {@link NumericalPropagatorBuilder}.
   *
   * @param initialState The initial spacecraft state.
   * @param odEpoch      The OD epoch. Nominally, the epoch of the last measurement.
   * @return The configured {@link NumericalPropagatorBuilder} to use.
   */
  static NumericalPropagatorBuilder createPropagatorBuilder(SpacecraftState initialState,
      AbsoluteDate odEpoch) {

    // TODO: make configurable integrator and propagator settings.
    ODEIntegratorBuilder integrator = new DormandPrince853IntegratorBuilder(0.0001, 300.0, 10.0);
    NumericalPropagatorBuilder builder = new NumericalPropagatorBuilder(initialState.getOrbit(),
        integrator, PositionAngle.MEAN, 1.0);

    NormalizedSphericalHarmonicsProvider gravityField = GravityFieldFactory.getNormalizedProvider(
        21, 21);
    HolmesFeatherstoneAttractionModel gravityModel = new HolmesFeatherstoneAttractionModel(
        AstroUtils.EARTH.getBodyFrame(), gravityField);

    ForceModel srpModel = new SolarRadiationPressure(CelestialBodyFactory.getSun(),
        Constants.IERS2010_EARTH_EQUATORIAL_RADIUS,
        new IsotropicRadiationSingleCoefficient(0.02, 1));

    builder.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getMoon()));
    builder.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getSun()));
    builder.addForceModel(gravityModel);
    builder.addForceModel(srpModel);

    builder.setAttitudeProvider(new InertialProvider(initialState.getFrame()));
    builder.setMass(initialState.getMass());
    builder.resetOrbit(initialState.getOrbit());

    // Shift to orbit to the OD epoch.
    if (!initialState.getDate().isEqualTo(odEpoch)) {
      SpacecraftState propagatedState = builder.buildPropagator(
          builder.getSelectedNormalizedParameters()).propagate(odEpoch);
      builder.resetOrbit(propagatedState.getOrbit());
    }

    return builder;
  }
}
