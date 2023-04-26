package org.cohere.od.utils;

import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.FramesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

/**
 * Utility class for Astro utilities and constants.
 */
public class AstroUtils {

  public static final OneAxisEllipsoid EARTH = new OneAxisEllipsoid(
      Constants.IERS2010_EARTH_EQUATORIAL_RADIUS,
      Constants.IERS2010_EARTH_FLATTENING,
      FramesFactory.getITRF(IERSConventions.IERS_2010, true));

  private AstroUtils() {
  }
}
