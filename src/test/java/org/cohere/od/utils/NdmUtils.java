/* Copyright 2002-2022 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.cohere.od.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.hipparchus.exception.LocalizedCoreFormats;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.RealMatrix;
import org.orekit.data.DataSource;
import org.orekit.errors.OrekitException;
import org.orekit.files.ccsds.definitions.BodyFacade;
import org.orekit.files.ccsds.definitions.CenterName;
import org.orekit.files.ccsds.definitions.FrameFacade;
import org.orekit.files.ccsds.definitions.TimeSystem;
import org.orekit.files.ccsds.ndm.ParserBuilder;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.files.ccsds.ndm.odm.CommonMetadata;
import org.orekit.files.ccsds.ndm.odm.KeplerianElements;
import org.orekit.files.ccsds.ndm.odm.StateVector;
import org.orekit.files.ccsds.ndm.odm.opm.Opm;
import org.orekit.files.ccsds.ndm.odm.opm.OpmData;
import org.orekit.files.ccsds.ndm.odm.opm.OpmParser;
import org.orekit.files.ccsds.ndm.odm.opm.OpmWriter;
import org.orekit.files.ccsds.section.Header;
import org.orekit.files.ccsds.section.Segment;
import org.orekit.files.ccsds.utils.generation.Generator;
import org.orekit.files.ccsds.utils.generation.XmlGenerator;
import org.orekit.frames.Frame;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * Utility class for parsing and generating CCSDS NDM files.
 * <p>
 * Based on the Orekit tutorial NDM class.
 */
public class NdmUtils {

  /**
   * Default file originator.
   */
  public static final String DEFAULT_ORIGINATOR = "OREKIT";

  /**
   * Default version for CCSDS files.
   */
  public static final double DEFAULT_VERSION = 2.0;

  /**
   * Private constructor for this utility class.
   */
  private NdmUtils() {
    // Do nothing
  }

  /**
   * Creates a CartesianCovariance data block.
   *
   * @param epoch  the epoch of the covariance
   * @param frame  the expression frame of the covariance
   * @param matrix the covariance matrix (expected to be 6x6)
   * @return the CartesianCovariance
   */
  public static CartesianCovariance createCartesianCovariance(AbsoluteDate epoch, Frame frame,
      RealMatrix matrix) {
    // Creates the CartesianCovariance from the given data
    final CartesianCovariance covariance = new CartesianCovariance(null);
    covariance.setEpoch(epoch);
    covariance.setReferenceFrame(FrameFacade.map(frame));
    // Sets the matrix
    for (int i = 0; i < matrix.getRowDimension(); ++i) {
      for (int j = 0; j <= i; ++j) {
        covariance.setCovarianceMatrixEntry(i, j, matrix.getEntry(i, j));
      }
    }
    // Returns the CartesianCovariance
    return covariance;
  }

  /**
   * Creates the common meta data.
   *
   * @param centerName     center name
   * @param objectName     spacecraft name
   * @param objectId       object identifier ("YYYY-NNNP" see CCSDS format)
   * @param referenceFrame reference frame
   * @return the common meta data
   */
  public static CommonMetadata createCommonMetadata(CenterName centerName, String objectName,
      String objectId, Frame referenceFrame) {
    CommonMetadata meta = new CommonMetadata();
    meta.setObjectName(objectName);
    meta.setObjectID(objectId);
    meta.setCenter(BodyFacade.create(centerName));
    meta.setReferenceFrame(FrameFacade.map(referenceFrame));
    meta.setTimeSystem(TimeSystem.UTC);
    return meta;
  }

  /**
   * Creates a header for a NDM.
   *
   * @return the header for the message
   */
  public static Header createHeader() {
    Header header = new Header(0);
    header.setFormatVersion(DEFAULT_VERSION);
    header.setCreationDate(
        new AbsoluteDate(AbsoluteDate.JAVA_EPOCH, System.currentTimeMillis() * 0.001));
    header.setOriginator(DEFAULT_ORIGINATOR);
    return header;
  }

  /**
   * Creates a KeplerianElements data block from a orbit.
   *
   * @param orbIn the orbit to get elements from
   * @return the KeplerianElements for an OPM
   */
  public static KeplerianElements createKeplerianElements(Orbit orbIn) {

    KeplerianOrbit orbit = (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(orbIn);
    KeplerianElements elements = new KeplerianElements();
    elements.setEpoch(orbit.getDate());
    elements.setA(orbit.getA());
    elements.setE(orbit.getE());
    elements.setI(orbit.getI());
    elements.setPa(orbit.getPerigeeArgument());
    elements.setRaan(orbit.getRightAscensionOfAscendingNode());
    elements.setAnomaly(orbit.getMeanAnomaly());
    elements.setAnomalyType(PositionAngle.MEAN);
    elements.setMu(orbit.getMu());

    return elements;
  }

  /**
   * Creates the segments for an OPM.
   *
   * @param metadata the metadata
   * @param data     the OPM data
   * @return the OPM segments
   */
  public static List<Segment<CommonMetadata, OpmData>> createOpmSegments(CommonMetadata metadata,
      OpmData data) {
    Segment<CommonMetadata, OpmData> segment = new Segment<>(metadata, data);
    List<Segment<CommonMetadata, OpmData>> segments = new ArrayList<>();
    segments.add(segment);
    return segments;
  }

  /**
   * Creates a StateVector data block from PV coordinates.
   *
   * @param pv the PV coordinates to get elements from
   * @return the StateVector
   */
  public static StateVector createStateVector(TimeStampedPVCoordinates pv) {
    StateVector sv = new StateVector();

    sv.setEpoch(pv.getDate());

    final Vector3D pos = pv.getPosition();
    sv.setP(0, pos.getX());
    sv.setP(1, pos.getY());
    sv.setP(2, pos.getZ());

    final Vector3D vel = pv.getVelocity();
    sv.setV(0, vel.getX());
    sv.setV(1, vel.getY());
    sv.setV(2, vel.getZ());

    Vector3D acc = pv.getAcceleration();
    if (!Vector3D.ZERO.equals(acc)) {
      sv.setA(0, acc.getX());
      sv.setA(1, acc.getY());
      sv.setA(2, acc.getZ());
    }

    return sv;
  }

  /**
   * Parses an OPM to a file.
   *
   * @param mu        central attraction coefficient
   * @param mass      spacecraft mass
   * @param inputFile the input file
   * @return the parsed OPM file
   */
  public static Opm parseOPM(double mu, double mass, File inputFile) {

    // Access the source
    DataSource source = new DataSource(inputFile.getName(),
        () -> new FileInputStream(new File(inputFile.getParentFile(), inputFile.getName())));

    // Create the parser
    OpmParser parser = new ParserBuilder().withMu(mu).withDefaultMass(mass).buildOpmParser();

    // Parse the message
    return parser.parseMessage(source);
  }

  /**
   * Writes an OPM to a file.
   *
   * @param writer     the OPM writer
   * @param opm        the OPM to write
   * @param outputFile the file to write to
   */
  public static void writeOPM(OpmWriter writer, Opm opm, File outputFile) throws IOException {

    try (BufferedWriter fileWriter = Files.newBufferedWriter(
        Paths.get(outputFile.getAbsolutePath()), StandardCharsets.UTF_8);
        Generator generator = new XmlGenerator(fileWriter, XmlGenerator.DEFAULT_INDENT,
            outputFile.getName(), true)) {

      writer.writeMessage(generator, opm);

    } catch (IOException ioe) {
      throw new OrekitException(LocalizedCoreFormats.SIMPLE_MESSAGE,
          "unable to write OPM file");
    }
  }

}
