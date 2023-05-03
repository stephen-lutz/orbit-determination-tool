package org.cohere.od.utils;

import org.orekit.attitudes.InertialProvider;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.forces.ForceModel;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.forces.radiation.IsotropicRadiationSingleCoefficient;
import org.orekit.forces.radiation.SolarRadiationPressure;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.conversion.DormandPrince853IntegratorBuilder;
import org.orekit.propagation.conversion.NumericalPropagatorBuilder;
import org.orekit.propagation.conversion.ODEIntegratorBuilder;
import org.orekit.utils.Constants;

public class PropagatorFactory {

  private static final double MAX_INTEGRATOR_STEP = 300.0;
  private static final double MIN_INTEGRATOR_STEP = 0.001;
  private static final double POSITION_ERROR = 10.0;
  private static final double SRP_AREA = 0.2;
  private static final double SRP_COEFFICIENT = 1.0;


  private PropagatorFactory() {
  }

  /**
   * Creates a {@link NumericalPropagatorBuilder} with the default force models:
   * <p>
   * Spherical harmonic 21x21 gravity model
   * <p>
   * Third body point-mass gravity from Sun and Moon
   * <p>
   * Solar radiation pressure with default area and Cr.
   *
   * @param initialState      The initial spacecraft state.
   * @param integratorBuilder The integrator builder to use.
   * @return The configured {@link NumericalPropagatorBuilder} to use.
   */
  public static NumericalPropagatorBuilder createDefaultPropagatorBuilder(
      SpacecraftState initialState, ODEIntegratorBuilder integratorBuilder) {

    NumericalPropagatorBuilder builder = new NumericalPropagatorBuilder(initialState.getOrbit(),
        integratorBuilder, PositionAngle.MEAN, 1.0);

    NormalizedSphericalHarmonicsProvider gravityField = GravityFieldFactory.getNormalizedProvider(
        21, 21);
    HolmesFeatherstoneAttractionModel gravityModel = new HolmesFeatherstoneAttractionModel(
        AstroUtils.EARTH.getBodyFrame(), gravityField);

    CelestialBody sun = CelestialBodyFactory.getSun();
    ForceModel srpModel = new SolarRadiationPressure(sun,
        Constants.IERS2010_EARTH_EQUATORIAL_RADIUS,
        new IsotropicRadiationSingleCoefficient(SRP_AREA, SRP_COEFFICIENT));

    builder.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getMoon()));
    builder.addForceModel(new ThirdBodyAttraction(sun));
    builder.addForceModel(gravityModel);
    builder.addForceModel(srpModel);

    builder.setAttitudeProvider(new InertialProvider(initialState.getFrame()));
    builder.setMass(initialState.getMass());
    builder.resetOrbit(initialState.getOrbit());

    return builder;
  }

  public static NumericalPropagatorBuilder createDefaultPropagatorBuilder(
      SpacecraftState initialState) {
    return createDefaultPropagatorBuilder(initialState,
        createIntegratorBuilder(MIN_INTEGRATOR_STEP, MAX_INTEGRATOR_STEP, POSITION_ERROR));
  }

  public static ODEIntegratorBuilder createIntegratorBuilder(double minStep, double maxStep,
      double positionError) {
    return new DormandPrince853IntegratorBuilder(minStep, maxStep, positionError);
  }

}
