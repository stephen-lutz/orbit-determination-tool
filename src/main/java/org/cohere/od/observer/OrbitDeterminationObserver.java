package org.cohere.od.observer;

import java.util.Locale;
import lombok.extern.log4j.Log4j2;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.util.FastMath;
import org.orekit.estimation.leastsquares.BatchLSEstimator;
import org.orekit.estimation.leastsquares.BatchLSObserver;
import org.orekit.estimation.measurements.AngularRaDec;
import org.orekit.estimation.measurements.EstimatedMeasurement;
import org.orekit.estimation.measurements.EstimationsProvider;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.orbits.Orbit;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterDriversList;

/**
 * Observer for the OrbitDetermination scheme.
 * <p>
 * Based on the Orekit tutorial OrbitDeterminationObserver class.
 */
@Log4j2
public class OrbitDeterminationObserver implements BatchLSObserver {

  /**
   * Format of the beginning of the first values' line.
   */
  private static final String FORMAT_0 = "    %2d         %2d                                 %16.12f     %s";
  /**
   * Format of the beginning of the header line.
   */
  private static final String FORMAT_HEADER = "iteration evaluations      ΔP(m)        ΔV(m/s)           RMS        nb Angular";
  /**
   * Format of the beginning of the other values' line.
   */
  private static final String FORMAT_L = "    %2d         %2d      %13.6f %12.9f %16.12f     %s";
  /**
   * Separator between parameters.
   */
  private static final String SEP = "  ";
  /**
   * Format for parameters' names.
   */
  private static final String PAR_STR = SEP + "%22s";
  /**
   * Format for parameters' values.
   */
  private static final String PAR_VAL = SEP + "%22.9f";
  /**
   * String format (needed to avoid Checkstyle errors...).
   */
  private static final String STR = "%s";
  /**
   * Previous PV value.
   */
  private PVCoordinates previousPV;

  /**
   * Constructor.
   *
   * @param initialGuess initial guess orbit
   * @param estimator    estimator to observe
   */
  public OrbitDeterminationObserver(Orbit initialGuess, BatchLSEstimator estimator) {
    this.previousPV = initialGuess.getPVCoordinates();
    String header = FORMAT_HEADER;
    header = addParametersNames(header,
        estimator.getOrbitalParametersDrivers(true),
        estimator.getPropagatorParametersDrivers(true),
        estimator.getMeasurementsParametersDrivers(true));
    log.info(header);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void evaluationPerformed(int iterationsCount, int evaluationsCount,
      Orbit[] orbits,
      ParameterDriversList estimatedOrbitalParameters,
      ParameterDriversList estimatedPropagatorParameters,
      ParameterDriversList estimatedMeasurementsParameters,
      EstimationsProvider evaluationsProvider,
      LeastSquaresProblem.Evaluation lspEvaluation) {

    PVCoordinates currentPV = orbits[0].getPVCoordinates();
    EvaluationCounter<AngularRaDec> angularCounter = new EvaluationCounter<>();

    for (int i = 0; i < evaluationsProvider.getNumber(); i++) {
      EstimatedMeasurement<?> estimatedMeasurement = evaluationsProvider.getEstimatedMeasurement(
          i);
      ObservedMeasurement<?> observed = estimatedMeasurement.getObservedMeasurement();
      String measurementType = observed.getMeasurementType();
      if (measurementType.equals(AngularRaDec.MEASUREMENT_TYPE)) {
        @SuppressWarnings("unchecked") EstimatedMeasurement<AngularRaDec> evaluation = (EstimatedMeasurement<AngularRaDec>) estimatedMeasurement;
        angularCounter.add(evaluation);
      }
    }

    // Print evaluation lines
    String line;
    if (evaluationsCount == 1) {
      line = String.format(Locale.US, FORMAT_0, iterationsCount, evaluationsCount,
          lspEvaluation.getRMS(), angularCounter.format(8));
    } else {
      line = String.format(Locale.US, FORMAT_L, iterationsCount, evaluationsCount,
          Vector3D.distance(previousPV.getPosition(), currentPV.getPosition()),
          Vector3D.distance(previousPV.getVelocity(), currentPV.getVelocity()),
          lspEvaluation.getRMS(), angularCounter.format(8));
    }

    line = addParametersValues(line, estimatedOrbitalParameters,
        estimatedPropagatorParameters, estimatedMeasurementsParameters);

    log.info(line);
    previousPV = currentPV;
  }

  /**
   * Add the parameters' names to a line.
   *
   * @param in                              the string to write in
   * @param estimatedOrbitalParameters      the estimated orbital parameters list
   * @param estimatedPropagatorParameters   the estimated propagation parameters list
   * @param estimatedMeasurementsParameters the estimated measurements parameters list
   * @return the string with the parameters added
   */
  private String addParametersNames(String in,
      ParameterDriversList estimatedOrbitalParameters,
      ParameterDriversList estimatedPropagatorParameters,
      ParameterDriversList estimatedMeasurementsParameters) {

    // Initialize out string
    String out = in;

    // Add orbital drivers
    for (ParameterDriver driver : estimatedOrbitalParameters.getDrivers()) {
      out = String.format(STR + PAR_STR, out, driver.getName());
    }

    // Add propagator parameters
    for (ParameterDriver driver : estimatedPropagatorParameters.getDrivers()) {
      String driverName = driver.getName();
      out = String.format(STR + PAR_STR, out, driverName);
    }

    // Add measurements parameters
    for (ParameterDriver driver : estimatedMeasurementsParameters.getDrivers()) {
      out = String.format(STR + PAR_STR, out, driver.getName());
    }

    return out;
  }

  /**
   * Add the parameters' values to a line.
   *
   * @param in                              the string to write in
   * @param estimatedOrbitalParameters      the estimated orbital parameters list
   * @param estimatedPropagatorParameters   the estimated propagation parameters list
   * @param estimatedMeasurementsParameters the estimated measurements parameters list
   * @return the string with the propagation parameters added
   */
  private String addParametersValues(String in,
      ParameterDriversList estimatedOrbitalParameters,
      ParameterDriversList estimatedPropagatorParameters,
      ParameterDriversList estimatedMeasurementsParameters) {

    String out = in;
    for (ParameterDriver driver : estimatedOrbitalParameters.getDrivers()) {
      out = String.format(Locale.US, STR + PAR_VAL, out, driver.getValue());
    }

    for (ParameterDriver driver : estimatedPropagatorParameters.getDrivers()) {
      out = String.format(Locale.US, STR + PAR_VAL, out, driver.getValue());
    }

    double driverValue;
    for (ParameterDriver driver : estimatedMeasurementsParameters.getDrivers()) {
      // rad to deg conversion for angular biases
      if (driver.getName().contains("/az bias") ||
          driver.getName().contains("/el bias")) {
        driverValue = FastMath.toDegrees(driver.getValue());
      } else {
        driverValue = driver.getValue();
      }
      out = String.format(Locale.US, STR + PAR_VAL, out, driverValue);
    }

    return out;
  }
}
