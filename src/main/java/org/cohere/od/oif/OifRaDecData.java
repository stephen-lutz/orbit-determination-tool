package org.cohere.od.oif;

import lombok.Data;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;

/**
 * Data contained in a line in an OIF measurement file.
 */
@Data
public class OifRaDecData {


  private double declination;
  private AbsoluteDate epoch;
  private Frame raDecFrame;
  private double rightAscension;
  private GeodeticPoint sensorLocation;
  private int targetId;

}
