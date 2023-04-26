package org.cohere.od.oif;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.cohere.od.utils.AstroUtils;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.estimation.measurements.AngularRaDec;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

// @formatter:off
/**
 * Parses OIF file and stores content.
 * <p>
 * Sample OIF:
 * <p>
 * CLASSIFICATION: UNCLASSIFIED
 * Observation Debug Output (RA,Dec, in GCRF; SenPos in GCRF) from Sensor 34 Chip 0 Filter "None" with base MJD = 60021 0.351950231481169
 * TargetID		Time(Seconds)	RA(Degrees)	Dec(Degrees)	SensorPosX	SensorPosY	SensorPosZ	SensorVelX	SensorVelY	SensorVelZ	VizMag	EstRange
 *  50008		  0.00	 -44.8827439541	  -2.7056903972	     6152.814973	     1679.541697	       37.682399	       -0.122474	        0.448663	        0.000259	  13.000000	    38453.944526
 *  50008		  6.20	 -44.8540245107	  -2.7048480984	     6152.055007	     1682.323236	       37.684007	       -0.122677	        0.448608	        0.000260	  13.000000	    38552.120189
 *  50008		 12.40	 -44.8406518142	  -2.7089404833	     6151.293783	     1685.104431	       37.685619	       -0.122879	        0.448552	        0.000260	  13.000000	    38531.393085
 *  50008		 18.60	 -44.8006225948	  -2.7041209706	     6150.531303	     1687.885283	       37.687233	       -0.123082	        0.448497	        0.000261	  13.000000	    38766.458827
 *  50008		 24.80	 -44.7786658250	  -2.6980410957	     6149.767565	     1690.665789	       37.688851	       -0.123285	        0.448441	        0.000261	  13.000000	    38177.578341
 *  50008		 31.00	 -44.7457829448	  -2.7133836654	     6149.002570	     1693.445949	       37.690471	       -0.123488	        0.448385	        0.000262	  13.000000	    38480.458008
 *  50008		 37.20	 -44.7313153891	  -2.6973859145	     6148.236318	     1696.225763	       37.692094	       -0.123690	        0.448329	        0.000262	  13.000000	    38462.115554
 *  50008		 43.40	 -44.7031348231	  -2.7095705626	     6147.468810	     1699.005231	       37.693719	       -0.123893	        0.448273	        0.000262	  13.000000	    38756.936729
 * NaN       		NaN       	NaN            	NaN            	NaN             	NaN             	NaN             	NaN             	NaN             	NaN             	NaN        	NaN
 */
// @formatter:on
public class OifHelper {

  private static final double KM_TO_M = 1000.0;
  private static final String NAN = "nan";

  private OifHelper() {
  }

  /**
   * Converts OIF data to Orekit measurement type: {@link ObservedMeasurement}.
   *
   * @param oifRaDecData The data to convert.
   * @param raDecSigmas  The standard deviations for right ascension and declination measurements,
   *                     respectively.
   * @return A list of {@link ObservedMeasurement}.
   */
  public static List<ObservedMeasurement<?>> convertOifData(
      List<OifRaDecData> oifRaDecData, double[] raDecSigmas) {

    double[] raDecWeights = new double[]{1.0, 1.0};

    List<ObservedMeasurement<?>> measurements = new ArrayList<>();
    ObservableSatellite satellite = new ObservableSatellite(0);
    for (OifRaDecData datum : oifRaDecData) {

      TopocentricFrame frame = new TopocentricFrame(AstroUtils.EARTH, datum.getSensorLocation(),
          "gsFrame");
      GroundStation groundStation = new GroundStation(frame);

      measurements.add(new AngularRaDec(groundStation, datum.getRaDecFrame(),
          datum.getEpoch(), new double[]{datum.getRightAscension(), datum.getDeclination()},
          raDecSigmas, raDecWeights, satellite));
    }

    return measurements;
  }

  /**
   * Parses an OIF measurement file.
   *
   * @param filePath The file path for the OIF.
   * @return A list of {@link OifRaDecData} from the file.
   * @throws IOException if the file cannot be read.
   */
  public static List<OifRaDecData> parseOifRaDecFile(Path filePath) throws IOException {

    final String MJD_KEYWORD = "MJD =";
    final int NUMBER_OF_HEADER_LINES = 3;

    List<String> allLines = Files.readAllLines(filePath);

    // TODO: parse frames from line 2.
    // Assume RA/Dec measurements and sensor positions are in GCRF frame.
    Frame raDecFrame = FramesFactory.getGCRF();
    Frame sensorPositionFrame = FramesFactory.getGCRF();

    String line2 = allLines.get(1);
    String mjdParts = line2.substring(line2.indexOf(MJD_KEYWORD) + MJD_KEYWORD.length()).trim();

    double mjdDay = Double.parseDouble(mjdParts.substring(0, mjdParts.indexOf(" ")));
    double mjdDayFraction = Double.parseDouble(mjdParts.substring(mjdParts.indexOf(" ")));
    AbsoluteDate oifEpoch = AbsoluteDate.MODIFIED_JULIAN_EPOCH.shiftedBy(
        (mjdDay + mjdDayFraction) * Constants.JULIAN_DAY);

    List<OifRaDecData> allData = new ArrayList<>();
    for (String line : allLines.stream().skip(NUMBER_OF_HEADER_LINES)
        .collect(Collectors.toList())) {

      if (line.toLowerCase().startsWith(NAN)) {
        continue;
      }

      List<String> lineParts = Arrays.stream(line.trim().split("\\s+"))
          .collect(Collectors.toList());

      int targetId = Integer.parseInt(lineParts.get(0));
      AbsoluteDate epoch = oifEpoch.shiftedBy(Double.parseDouble(lineParts.get(1)));
      double rightAscension = Math.toRadians(Double.parseDouble(lineParts.get(2)));
      double declination = Math.toRadians(Double.parseDouble(lineParts.get(3)));
      double sensorPositionX = Double.parseDouble(lineParts.get(4));
      double sensorPositionY = Double.parseDouble(lineParts.get(5));
      double sensorPositionZ = Double.parseDouble(lineParts.get(6));
      Vector3D sensorPosition = new Vector3D(sensorPositionX, sensorPositionY,
          sensorPositionZ).scalarMultiply(KM_TO_M);
      GeodeticPoint sensorLocation = AstroUtils.EARTH.transform(sensorPosition, sensorPositionFrame,
          epoch);

      OifRaDecData data = new OifRaDecData();
      data.setTargetId(targetId);
      data.setEpoch(epoch);
      data.setRightAscension(rightAscension);
      data.setDeclination(declination);
      data.setRaDecFrame(raDecFrame);
      data.setSensorLocation(sensorLocation);

      allData.add(data);
    }

    return allData;
  }

}
