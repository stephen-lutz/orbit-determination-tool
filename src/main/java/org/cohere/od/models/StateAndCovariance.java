package org.cohere.od.models;

import lombok.Value;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;

/**
 * Container class for spacecraft state and covariance.
 */
@Value
public class StateAndCovariance {

  StateCovariance covariance;
  SpacecraftState state;
}
