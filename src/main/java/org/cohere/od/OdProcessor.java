package org.cohere.od;

import java.util.List;
import lombok.NonNull;
import org.cohere.od.models.StateAndCovariance;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.propagation.SpacecraftState;

public interface OdProcessor {

  StateAndCovariance processMeasurements(@NonNull SpacecraftState initialState,
      @NonNull List<ObservedMeasurement<?>> measurements);

}
