package org.cohere.od.oif;

import lombok.Value;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;

/**
 * Data contained in a line in an OIF measurement file.
 */
@Value
public class OifRaDecData {


  double declination;
  AbsoluteDate epoch;
  Frame raDecFrame;
  double rightAscension;
  GeodeticPoint sensorLocation;
  int targetId;

}
