package org.cohere.od.models;

import lombok.AllArgsConstructor;
import lombok.Data;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;

/**
 * Container class for spacecraft state and covariance.
 */
@Data
@AllArgsConstructor
public class StateAndCovariance {

  StateCovariance covariance;
  SpacecraftState state;
}
