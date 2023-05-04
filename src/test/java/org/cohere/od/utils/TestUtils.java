package org.cohere.od.utils;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.RealMatrix;
import org.junit.jupiter.api.Assertions;
import org.orekit.utils.TimeStampedPVCoordinates;

public class TestUtils {

  private static final double TOLERANCE = 1e-12;

  private TestUtils() {
  }

  public static void assertAreEqual(Vector3D expected, Vector3D actual, double tolerance) {
    Assertions.assertEquals(expected.getX(), actual.getX(), tolerance);
    Assertions.assertEquals(expected.getY(), actual.getY(), tolerance);
    Assertions.assertEquals(expected.getZ(), actual.getZ(), tolerance);
  }

  public static void assertAreEqual(TimeStampedPVCoordinates expected,
      TimeStampedPVCoordinates actual) {
    Assertions.assertTrue(expected.getDate().isCloseTo(actual, TOLERANCE));
    assertAreEqual(expected.getPosition(), actual.getPosition(), TOLERANCE);
    assertAreEqual(expected.getVelocity(), actual.getVelocity(), TOLERANCE);
  }

  public static void assertAreEqual(RealMatrix expected, RealMatrix actual) {

    Assertions.assertEquals(expected.getRowDimension(), actual.getRowDimension());
    Assertions.assertEquals(expected.getColumnDimension(), actual.getColumnDimension());

    for (int row = 0; row < expected.getRowDimension(); row++) {
      for (int column = 0; column < expected.getColumnDimension(); column++) {
        Assertions.assertEquals(expected.getEntry(row, column), actual.getEntry(row, column),
            TOLERANCE);
      }
    }
  }

}
