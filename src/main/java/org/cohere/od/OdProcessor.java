package org.cohere.od;

import java.util.List;
import lombok.NonNull;
import org.cohere.od.models.StateAndCovariance;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.propagation.SpacecraftState;

/**
 * Interface for orbit determination processors.
 * <p>
 * The processor will ingest an initial state and list of measurements, perform an OD, and produce a
 * resulting state and covariance.
 */
public interface OdProcessor {

  StateAndCovariance processMeasurements(@NonNull SpacecraftState initialState,
      @NonNull List<ObservedMeasurement<?>> measurements);

}
