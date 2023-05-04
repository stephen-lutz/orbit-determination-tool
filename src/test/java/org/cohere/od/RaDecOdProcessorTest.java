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
import org.hipparchus.linear.MatrixUtils;
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
import org.orekit.propagation.conversion.OrbitDeterminationPropagatorBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
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
  private static TimeScale UTC = null;

  @BeforeAll
  public static void initializeOrekit() {
    File orekitData = new File("./orekit-data");
    DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
    manager.addProvider(new DirectoryCrawler(orekitData));
    UTC = TimeScalesFactory.getUTC();
  }

  @Test
  void testProcessGeneratedMeasurementsAf3() throws IOException {

    // @formatter:off
    AbsoluteDate odEpoch = new AbsoluteDate("2023-03-18T12:04:00.000Z", UTC);
    TimeStampedPVCoordinates expectedPv = new TimeStampedPVCoordinates(odEpoch,
        new Vector3D(4.065756782602535E7, 1.2430467597207164E7, 167940.87638002576),
        new Vector3D(-867.7215959246972, 2930.563389688151, 168.2581788419025));
    RealMatrix expectedCovariance = MatrixUtils.createRealMatrix(new double[][]{
        {1.6076949424439210e+11, 2.6467442018919693e+10, 5.8352873573301740e+08, 1.5441872589521830e+07, -1.8621414386847627e+06, -3.0228896775963020e+05},
        {2.6467442018920036e+10, 4.3590233969518160e+09, 9.6117235129955740e+07, 2.5421786582802410e+06, -3.0657400187425374e+05, -4.9766479618869570e+04},
        {5.8352873573306920e+08, 9.6117235129963030e+07, 2.9477664720739084e+06, 5.6046333634969335e+04, -6.7591397809909020e+03, -1.0959413032722118e+03},
        {1.5441872589521784e+07, 2.5421786582801994e+06, 5.6046333634964180e+04, 1.4831903867660110e+03, -1.7885656539752068e+02, -2.9034614208587450e+01},
        {-1.8621414386848032e+06, -3.0657400187425636e+05, -6.7591397809904520e+03, -1.7885656539752512e+02, 2.1570095342342828e+01, 3.5009192053753930e+00},
        {-3.0228896775962337e+05, -4.9766479618867794e+04, -1.0959413032720897e+03, -2.9034614208586880e+01, 3.5009192053752380e+00, 5.7691670502378560e-01}
    });
    // @formatter:on

    testProcessMeasurements(AF3, true, expectedPv, expectedCovariance);
  }

  @Test
  void testProcessGeneratedMeasurementsCp1() throws IOException {

    // @formatter:off
    AbsoluteDate odEpoch = new AbsoluteDate("2023-03-18T12:18:20.000Z", UTC);
    TimeStampedPVCoordinates expectedPv = new TimeStampedPVCoordinates(odEpoch,
        new Vector3D(-4.205275019328148E7, 3663853.0834480855, -372257.7833810879),
        new Vector3D(-269.98387966286515, -3060.473802043133, -54.78626405314519));
    RealMatrix expectedCovariance = MatrixUtils.createRealMatrix(new double[][]{
        {1.0289236399170946e+09, -1.7301467202280113e+08, 1.3399037443561802e+07, 8.6767237746715520e+04, -4.7734813526751670e+04, 9.2976970291187940e+02},
        {-1.7301467202280140e+08, 3.6277724629696610e+07, -2.3092213466648840e+06, -1.4882114804085024e+04, 8.1342156313884080e+03, -1.4876641616952540e+02},
        {1.3399037443561753e+07, -2.3092213466648697e+06, 3.6985344543715804e+06, 1.1238797130702421e+03, -6.2647888926286510e+02, 1.5008377411966290e+01},
        {8.6767237746715500e+04, -1.4882114804084997e+04, 1.1238797130702453e+03, 7.3420957996621750e+00, -4.0254451764814645e+00, 7.7950933880361880e-02},
        {-4.7734813526751640e+04, 8.1342156313883840e+03, -6.2647888926286700e+02, -4.0254451764814630e+00, 2.2179351035852086e+00, -4.3612659742936544e-02},
        {9.2976970291185200e+02, -1.4876641616952048e+02, 1.5008377411965968e+01, 7.7950933880359560e-02, -4.3612659742935295e-02, 3.8005144948521745e-02}
    });
    // @formatter:on

    testProcessMeasurements(CP1, true, expectedPv, expectedCovariance);
  }

  @Test
  void testProcessGeneratedMeasurementsSa2() throws IOException {

    // @formatter:off
    AbsoluteDate odEpoch = new AbsoluteDate("2023-03-18T12:16:10.000Z", UTC);
    TimeStampedPVCoordinates expectedPv = new TimeStampedPVCoordinates(odEpoch,
        new Vector3D(1.9210039652757324E7, -3.756800846680071E7, -367099.41026818793),
        new Vector3D(2737.019370532968, 1397.4515069369206, -55.543912978984686));
    RealMatrix expectedCovariance = MatrixUtils.createRealMatrix(new double[][]{
        {1.1990618398533331e+08, -1.9348231089502493e+08, -1.5273180520099397e+06, 1.3998206747943202e+03, -2.0431922719861600e+04, -3.1502895379237770e+02},
        {-1.9348231089502698e+08, 3.3873113442698900e+08, 2.7731087404994047e+06, -2.4960342787613820e+03, 3.5564965881342050e+04, 5.1731262511020610e+02},
        {-1.5273180520099334e+06, 2.7731087404993660e+06, 3.6138798263230654e+06, -7.5068890637461290e+00, 2.8214053912641350e+02, 6.7811403540951050e+00},
        {1.3998206747943586e+03, -2.4960342787614168e+03, -7.5068890637466740e+00, 2.8217111875118460e-02, -2.6921371212875406e-01, -3.3477753451532084e-03},
        {-2.0431922719861870e+04, 3.5564965881342020e+04, 2.8214053912641710e+02, -2.6921371212875017e-01, 3.7419059686116025e+00, 5.4936466493510354e-02},
        {-3.1502895379238010e+02, 5.1731262511020400e+02, 6.7811403540951405e+00, -3.3477753451531390e-03, 5.4936466493510173e-02, 3.8621982079724790e-02}
    });
    // @formatter:on

    testProcessMeasurements(SA2, true, expectedPv, expectedCovariance);
  }

  @Test
  void testProcessOifMeasurementsAf3() throws IOException {

    // @formatter:off
    AbsoluteDate odEpoch = new AbsoluteDate("2023-03-18T10:13:55.46599980926514Z", UTC);
    TimeStampedPVCoordinates expectedPv = new TimeStampedPVCoordinates(odEpoch,
        new Vector3D(4.148568388762964E7, -7463914.111923715, -914059.1874955879),
        new Vector3D(547.2706590865037, 3021.6868619484885, 155.84892039809355));
    RealMatrix expectedCovariance = MatrixUtils.createRealMatrix(new double[][]{
        {6.3926835438898500e+06, -1.8917451334524346e+06, -1.8200930181494862e+05, 6.2273660277829130e+02, -2.6045634692086435e+02, -6.2411803409510570e+00},
        {-1.8917451334524767e+06, 1.0124897520771717e+06, 7.2757937184078110e+04, -1.9522176830688628e+02, 9.4488522596690960e+01, 3.8136390877436823e+00},
        {-1.8200930181494790e+05, 7.2757937184076670e+04, 2.4315545083013453e+05, -1.7904726908766330e+01, 9.5207249361717810e+00, -4.4581116849921560e+00},
        {6.2273660277829460e+02, -1.9522176830688318e+02, -1.7904726908766510e+01, 6.3126385663087620e-02, -2.4608396547154292e-02, -4.4938494218242696e-04},
        {-2.6045634692086450e+02, 9.4488522596689320e+01, 9.5207249361718220e+00, -2.4608396547154174e-02, 1.2000796943224630e-02, 3.2310160335924013e-04},
        {-6.2411803409512930e+00, 3.8136390877437120e+00, -4.4581116849921445e+00, -4.4938494218244670e-04, 3.2310160335925000e-04, 1.9090639352037433e-03}
    });
    // @formatter:on

    testProcessMeasurements(AF3, false, expectedPv, expectedCovariance);
  }

  @Test
  void testProcessOifMeasurementsCp1() throws IOException {

    // @formatter:off
    AbsoluteDate odEpoch = new AbsoluteDate("2023-03-18T16:41:25.566Z", UTC);
    TimeStampedPVCoordinates expectedPv = new TimeStampedPVCoordinates(odEpoch,
        new Vector3D(-2.0267884848468445E7, -3.696741054899974E7, -837312.577751095),
        new Vector3D(2695.9798544858536, -1477.827690313697, 2.953093935463171));
    RealMatrix expectedCovariance = MatrixUtils.createRealMatrix(new double[][]{
        {5.5634042279547660e+06, 6.3788280389256380e+06, -4.8577277331542070e+04, 6.3631681893512960e+02, 5.7087724433767410e+02, 1.0504037860032852e+01},
        {6.3788280389254960e+06, 1.2111402475276040e+07, -1.2619099496620779e+05, 9.7172129195653400e+02, 8.9473452718387810e+02, 1.2343892034693049e+01},
        {-4.8577277331541200e+04, -1.2619099496620827e+05, 1.2811878993579117e+06, -7.1492450643593700e+00, -8.0944266863619540e+00, 1.3155729066900205e+01},
        {6.3631681893512230e+02, 9.7172129195654220e+02, -7.1492450643593860e+00, 8.5382573800690400e-02, 7.7269608382469280e-02, 1.1598384905079381e-03},
        {5.7087724433766720e+02, 8.9473452718388520e+02, -8.0944266863619720e+00, 7.7269608382469270e-02, 7.3626265686895510e-02, 1.0198270155445210e-03},
        {1.0504037860032936e+01, 1.2343892034693432e+01, 1.3155729066900207e+01, 1.1598384905079629e-03, 1.0198270155445434e-03, 5.5591158227775660e-03}
    });
    // @formatter:on

    testProcessMeasurements(CP1, false, expectedPv, expectedCovariance);
  }

  @Test
  void testProcessOifMeasurementsSa2() throws IOException {

    // @formatter:off
    AbsoluteDate odEpoch = new AbsoluteDate("2023-03-18T11:33:55.46599980926514Z", UTC);
    TimeStampedPVCoordinates expectedPv = new TimeStampedPVCoordinates(odEpoch,
        new Vector3D(1.2176804471140701E7, -4.036679100089015E7, -227153.14572922845),
        new Vector3D(2942.8109968178715, 888.6652428412206, -59.25652715150254));
    RealMatrix expectedCovariance = MatrixUtils.createRealMatrix(new double[][]{
        {3.2291432817787830e+06, -2.6847495530435610e+06, 3.3166698636846260e+04, 9.8042168809386850e+01, -3.0098482224185165e+02, 6.3256916688202650e+00},
        {-2.6847495530438660e+06, 5.5003454952210754e+07, 1.2718793595028213e+05, -3.9302495929570540e+02, 5.7266287014740260e+03, -9.0298893695886580e+01},
        {3.3166698636845810e+04, 1.2718793595028523e+05, 1.5896797510321895e+06, 3.2474365938777967e+00, 1.0618771012156875e+01, -6.8096078714927610e+00},
        {9.8042168809387630e+01, -3.9302495929570010e+02, 3.2474365938778336e+00, 1.1949961198530349e-02, -4.5980805421676410e-02, 1.0625623597780710e-03},
        {-3.0098482224187586e+02, 5.7266287014740260e+03, 1.0618771012156547e+01, -4.5980805421676964e-02, 5.9974842546798620e-01, -9.4936599602708890e-03},
        {6.3256916688204290e+00, -9.0298893695887460e+01, -6.8096078714927590e+00, 1.0625623597780860e-03, -9.4936599602709800e-03, 8.6152596830157500e-03}
    });
    // @formatter:on

    testProcessMeasurements(SA2, false, expectedPv, expectedCovariance);
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
    AbsoluteDate initialEpoch = new AbsoluteDate("2023-03-18T00:00:00.000", UTC);
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
    AbsoluteDate initialEpoch = new AbsoluteDate("2023-03-18T00:14:26.889000", UTC);
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
    AbsoluteDate initialEpoch = new AbsoluteDate("2023-03-18T00:12:14.305", UTC);
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

  private void testProcessMeasurements(String objectName, boolean isToGenerateMeasurements,
      TimeStampedPVCoordinates expectedPv, RealMatrix expectedCovariance) throws IOException {

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
    TestUtils.assertAreEqual(expectedPv,
        estimatedStateAndCovariance.getState().getPVCoordinates(FramesFactory.getGCRF()));
    TestUtils.assertAreEqual(expectedCovariance, estimatedStateAndCovariance.getCovariance()
        .getMatrix());

    // Create an OPM and OEM based on the estimated orbit and covariance.
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