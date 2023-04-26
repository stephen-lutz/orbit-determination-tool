package org.cohere.od;

import lombok.AllArgsConstructor;
import lombok.Data;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;

@Data
@AllArgsConstructor
public class StateAndCovariance {

  StateCovariance covariance;
  SpacecraftState state;
}
