package org.cohere.od;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import lombok.extern.log4j.Log4j2;
import org.cohere.od.generation.MeasurementGenerator;
import org.cohere.od.models.StateAndCovariance;
import org.cohere.od.oif.OifHelper;
import org.cohere.od.oif.OifRaDecData;
import org.cohere.od.utils.AstroUtils;
import org.cohere.od.utils.EstimatorFactory;
import org.cohere.od.utils.NdmUtils;
import org.cohere.od.utils.PropagatorFactory;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.random.RandomDataGenerator;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.estimation.leastsquares.BatchLSEstimator;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.files.ccsds.definitions.BodyFacade;
import org.orekit.files.ccsds.definitions.CenterName;
import org.orekit.files.ccsds.definitions.FrameFacade;
import org.orekit.files.ccsds.definitions.TimeSystem;
import org.orekit.files.ccsds.ndm.WriterBuilder;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.files.ccsds.ndm.odm.oem.OemMetadata;
import org.orekit.files.ccsds.ndm.odm.oem.StreamingOemWriter;
import org.orekit.files.ccsds.ndm.odm.opm.Opm;
import org.orekit.files.ccsds.ndm.odm.opm.OpmData;
import org.orekit.files.ccsds.utils.generation.Generator;
import org.orekit.files.ccsds.utils.generation.KvnGenerator;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.conversion.ODEIntegratorBuilder;
import org.orekit.propagation.conversion.OrbitDeterminationPropagatorBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.TimeStampedPVCoordinates;

@Log4j2
class RaDecOdProcessorTest {

  private static final String AF3 = "AF3";
  private static final String CP1 = "CP1";
  private static final double KM_TO_M = 1000.0;
  private static final String SA2 = "SA2";
  private static final double SPACECRAFT_MASS = 500.0;
  private static final String TEST_OUTPUT_PATH = "test_output";
  private static final String TEST_RESOURCES_PATH = "src/test/resources";

  @BeforeAll
  public static void initializeOrekit() {
    File orekitData = new File("./orekit-data");
    DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
    manager.addProvider(new DirectoryCrawler(orekitData));
  }

  @Test
  void testProcessMeasurementsAf3() throws IOException {
    testProcessMeasurements(AF3, true);
    testProcessMeasurements(AF3, false);
  }

  @Test
  void testProcessMeasurementsCp1() throws IOException {
    testProcessMeasurements(CP1, true);
    testProcessMeasurements(CP1, false);
  }

  @Test
  void testProcessMeasurementsSa2() throws IOException {
    testProcessMeasurements(SA2, true);
    testProcessMeasurements(SA2, false);
  }

  private GroundStation createDiegoGarcia() {
    TopocentricFrame gsFrame = new TopocentricFrame(AstroUtils.EARTH,
        new GeodeticPoint(Math.toRadians(0.465765), Math.toRadians(73.2162), -94.1783),
        "DiegoGarcia");
    return new GroundStation(gsFrame);
  }

  private GroundStation createEglin() {
    TopocentricFrame gsFrame = new TopocentricFrame(AstroUtils.EARTH,
        new GeodeticPoint(Math.toRadians(30.476), Math.toRadians(-86.5857), -21.24), "Eglin");
    return new GroundStation(gsFrame);
  }

  private SpacecraftState createInitialStateAf3() {
    AbsoluteDate initialEpoch = new AbsoluteDate("2023-03-18T00:00:00.000",
        TimeScalesFactory.getUTC());
    Vector3D initialPosition = new Vector3D(-4.062348841462340e+04, -1.131092510063584e+04,
        -1.074224879016023e+02).scalarMultiply(KM_TO_M);
    Vector3D initialVelocity = new Vector3D(8.243020099302774e-01, -2.957143748313477e+00,
        -1.692574154589104e-01).scalarMultiply(KM_TO_M);
    TimeStampedPVCoordinates initialPv = new TimeStampedPVCoordinates(initialEpoch, initialPosition,
        initialVelocity);

    Orbit orbit = new CartesianOrbit(initialPv, FramesFactory.getGCRF(),
        Constants.IERS2010_EARTH_MU);
    return new SpacecraftState(orbit, SPACECRAFT_MASS);
  }

  private SpacecraftState createInitialStateCp1() {
    AbsoluteDate initialEpoch = new AbsoluteDate("2023-03-18T00:14:26.889000",
        TimeScalesFactory.getUTC());
    Vector3D initialPosition = new Vector3D(4.189974449058950e+04, -4.713177927901786e+03,
        3.562232987686665e+02).scalarMultiply(KM_TO_M);
    Vector3D initialVelocity = new Vector3D(3.433104066667355e-01, 3.055016180848359e+00,
        5.550461291772908e-02).scalarMultiply(KM_TO_M);
    TimeStampedPVCoordinates initialPv = new TimeStampedPVCoordinates(initialEpoch, initialPosition,
        initialVelocity);

    Orbit orbit = new CartesianOrbit(initialPv, FramesFactory.getGCRF(),
        Constants.IERS2010_EARTH_MU);
    return new SpacecraftState(orbit, SPACECRAFT_MASS);
  }

  private SpacecraftState createInitialStateSa2() {
    AbsoluteDate initialEpoch = new AbsoluteDate("2023-03-18T00:12:14.305",
        TimeScalesFactory.getUTC());
    Vector3D initialPosition = new Vector3D(-1.824071309541569e+04, 3.801466820067245e+04,
        3.508492406457166e+02).scalarMultiply(KM_TO_M);
    Vector3D initialVelocity = new Vector3D(-2.771468115015459e+00, -1.330284523529563e+00,
        5.610314702236215e-02).scalarMultiply(KM_TO_M);
    TimeStampedPVCoordinates initialPv = new TimeStampedPVCoordinates(initialEpoch, initialPosition,
        initialVelocity);

    Orbit orbit = new CartesianOrbit(initialPv, FramesFactory.getGCRF(),
        Constants.IERS2010_EARTH_MU);
    return new SpacecraftState(orbit, SPACECRAFT_MASS);
  }

  private GroundStation createMaui() {
    TopocentricFrame gsFrame = new TopocentricFrame(AstroUtils.EARTH,
        new GeodeticPoint(Math.toRadians(20.6924), Math.toRadians(-156.309), 2119.62), "Maui");
    return new GroundStation(gsFrame);
  }

  private List<ObservedMeasurement<?>> generateTestMeasurements(SpacecraftState initialState,
      GroundStation groundStation, double[] raDecSigmas) {

    OrbitDeterminationPropagatorBuilder propagatorBuilder =
        PropagatorFactory.createDefaultPropagatorBuilder(initialState);
    Propagator propagator = propagatorBuilder.buildPropagator(
        propagatorBuilder.getSelectedNormalizedParameters());

    double burstInterval = 3600 * 6.0;  // 6 hours between sets of measurements
    double measurementStepSize = 10.0;  // 10 seconds between measurements
    double[] raDecWeights = new double[]{1.0, 1.0}; // equally weighted and unscaled
    RandomDataGenerator random = new RandomDataGenerator(123456);

    MeasurementGenerator measurementGenerator = new MeasurementGenerator(propagator, random);
    AbsoluteDate measStart = initialState.getDate().shiftedBy(60.0);
    AbsoluteDate measStop = measStart.shiftedBy(60.0);
    List<ObservedMeasurement<?>> measurements = new ArrayList<>(
        measurementGenerator.generateRaDecMeasurements(
            groundStation, raDecSigmas, raDecWeights, measStart, measStop, measurementStepSize));

    measurementGenerator = new MeasurementGenerator(propagator, random);
    measStart = measStop.shiftedBy(burstInterval);
    measStop = measStart.shiftedBy(60.0);
    measurements.addAll(measurementGenerator.generateRaDecMeasurements(
        groundStation, raDecSigmas, raDecWeights, measStart, measStop, measurementStepSize));

    measurementGenerator = new MeasurementGenerator(propagator, random);
    measStart = measStop.shiftedBy(burstInterval);
    measStop = measStart.shiftedBy(60.0);
    measurements.addAll(measurementGenerator.generateRaDecMeasurements(
        groundStation, raDecSigmas, raDecWeights, measStart, measStop, measurementStepSize));

    return measurements;

  }

  private void testProcessMeasurements(String objectName, boolean isToGenerateMeasurements)
      throws IOException {

    SpacecraftState initialState;
    GroundStation groundStation;
    double[] raDecSigmas;
    String objectId;
    Path oifRoot = Path.of(TEST_RESOURCES_PATH, "oif");

    if (objectName.equalsIgnoreCase(CP1)) {

      initialState = createInitialStateCp1();
      groundStation = createMaui();
      double sigma = Math.toRadians(0.01);
      raDecSigmas = new double[]{sigma, sigma};
      objectId = "50013";
      oifRoot = Path.of(oifRoot.toString(), "cp1");

    } else if (objectName.equalsIgnoreCase(SA2)) {

      initialState = createInitialStateSa2();
      groundStation = createEglin();
      double sigma = Math.toRadians(0.01);
      raDecSigmas = new double[]{sigma, sigma};
      objectId = "50036";
      oifRoot = Path.of(oifRoot.toString(), "sa2");

    } else if (objectName.equalsIgnoreCase(AF3)) {

      initialState = createInitialStateAf3();
      groundStation = createDiegoGarcia();
      double sigma = Math.toRadians(0.005);
      raDecSigmas = new double[]{sigma, sigma};
      objectId = "50008";
      oifRoot = Path.of(oifRoot.toString(), "af3");

    } else {
      throw new IllegalArgumentException("Unsupported test case for object: " + objectName);
    }

    List<ObservedMeasurement<?>> measurements = new ArrayList<>();
    if (isToGenerateMeasurements) {

      log.debug("Generating RA/Dec measurements for {} to {}.",
          groundStation.getBaseFrame().getName(), objectName);
      measurements = generateTestMeasurements(initialState, groundStation, raDecSigmas);

    } else {

      File[] oifFiles = oifRoot.toFile().listFiles((d, name) -> name.endsWith(".oif"));
      assertNotNull(oifFiles);
      for (File oifFile : oifFiles) {
        log.debug("Reading OIF file: {}", oifFile.toPath().toRealPath().toString());
        List<OifRaDecData> raDecData = OifHelper.parseOifRaDecFile(oifFile.toPath());
        measurements.addAll(OifHelper.convertOifData(raDecData, raDecSigmas));
      }

    }

    // Create the OD processor.
    OrbitDeterminationPropagatorBuilder propagatorBuilder = PropagatorFactory.createDefaultPropagatorBuilder(
        initialState);
    BatchLSEstimator estimator = EstimatorFactory.createBatchLsEstimator(propagatorBuilder,
        measurements);
    OdProcessor processor = new RaDecOdProcessor(estimator);

    // Process the measurements.
    StateAndCovariance estimatedStateAndCovariance = processor.processMeasurements(initialState,
        measurements);
    assertNotNull(estimatedStateAndCovariance);

    // Compare final state and covariance to expected values.

    // Create an OPM and OEM based on the estimated orbit and covariance
    Files.createDirectories(Path.of(TEST_OUTPUT_PATH));
    WriterBuilder ndmWriterBuilder = new WriterBuilder().withConventions(IERSConventions.IERS_2010);
    objectName = isToGenerateMeasurements ? objectName + "_generatedMeas" : objectName + "_oifData";
    writeOpmFile(objectName, objectId, ndmWriterBuilder, estimatedStateAndCovariance.getState(),
        estimatedStateAndCovariance.getCovariance().getMatrix());
    writeOemFile(objectName, objectId, initialState.getDate(),
        estimatedStateAndCovariance.getState(), ndmWriterBuilder);
  }

  /**
   * Write the output OEM file.
   *
   * @param objectName       Object name for OEM
   * @param objectId         Object ID for OEM
   * @param startTime        start time of the OEM (initial state epoch)
   * @param estimatedState   estimated state
   * @param ndmWriterBuilder builder for NDM files
   */
  private void writeOemFile(String objectName, String objectId, AbsoluteDate startTime,
      SpacecraftState estimatedState, WriterBuilder ndmWriterBuilder)
      throws IOException {

    // OEM meta data
    OemMetadata metadataTemplate = new OemMetadata(4);
    metadataTemplate.setObjectName(objectName);
    metadataTemplate.setObjectID(objectId);
    metadataTemplate.setCenter(
        new BodyFacade("EARTH", CelestialBodyFactory.getCelestialBodies().getEarth()));
    metadataTemplate.setReferenceFrame(FrameFacade.map(estimatedState.getFrame()));
    metadataTemplate.setTimeSystem(TimeSystem.UTC);

    // Back propagate to the initial state epoch.
    OrbitDeterminationPropagatorBuilder builder =
        PropagatorFactory.createDefaultPropagatorBuilder(estimatedState);
    Propagator propagator = builder.buildPropagator(builder.getSelectedNormalizedParameters());
    propagator.propagate(startTime);

    double duration = estimatedState.getDate().durationFrom(propagator.getInitialState().getDate());

    // Set the OEM writer to the propagator
    File oemFile = Path.of(TEST_OUTPUT_PATH, objectName + ".oem").toFile();
    log.debug("Writing estimated OEM to {}", oemFile.getAbsolutePath());
    try (BufferedWriter fileWriter = Files.newBufferedWriter(Paths.get(oemFile.getAbsolutePath()),
        StandardCharsets.UTF_8);
        Generator generator = new KvnGenerator(fileWriter, 25, oemFile.getName(), 60);
        StreamingOemWriter sw = new StreamingOemWriter(generator,
            ndmWriterBuilder.buildOemWriter(), NdmUtils.createHeader(), metadataTemplate)) {

      // Let the propagator generate the ephemeris
      propagator.getMultiplexer().clear();
      propagator.getMultiplexer().add(60.0, sw.newSegment());
      propagator.propagate(propagator.getInitialState().getDate().shiftedBy(duration));

    }

  }

  /**
   * Write the output OPM file.
   *
   * @param objectName       Object name for OPM
   * @param ndmWriterBuilder builder for NDM files
   * @param estimated        estimated orbit
   * @param covEstimated     estimated covariance
   */
  private void writeOpmFile(String objectName, String objectId, WriterBuilder ndmWriterBuilder,
      SpacecraftState estimated, RealMatrix covEstimated) throws IOException {

    // Covariance data
    CartesianCovariance covariance = NdmUtils.createCartesianCovariance(estimated.getDate(),
        estimated.getFrame(), covEstimated);

    // Create the data
    OpmData data = new OpmData(NdmUtils.createStateVector(estimated.getPVCoordinates()),
        NdmUtils.createKeplerianElements(estimated.getOrbit()),
        null,
        covariance,
        new ArrayList<>(), // Empty list of maneuvers
        null,
        estimated.getMass());

    // Opm
    Opm opm = new Opm(NdmUtils.createHeader(), NdmUtils.createOpmSegments(
        NdmUtils.createCommonMetadata(CenterName.EARTH, objectName, objectId, estimated.getFrame()),
        data), IERSConventions.IERS_2010, DataContext.getDefault(), Constants.WGS84_EARTH_MU);

    // Write the file
    File opmFile = Path.of(TEST_OUTPUT_PATH, objectName + ".opm").toFile();
    log.debug("Writing estimated OPM to {}", opmFile.getAbsolutePath());
    NdmUtils.writeOPM(ndmWriterBuilder.buildOpmWriter(), opm, opmFile);
  }
}