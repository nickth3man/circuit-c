/*
 * auto_transmission.h — automatic gear selection and arcade direction-swap state machine.
 *
 * A pre-physics gating stage: it reads the entrant's controller output and rewrites the pedals
 * on it, so the assist can never be confused with what the driver asked for.
 */
#ifndef CIRCUIT_AUTO_TRANSMISSION_H
#define CIRCUIT_AUTO_TRANSMISSION_H

#include <stdbool.h>

#include "physics/vehicle.h"
#include "game/controller_output.h"

void auto_transmission_update(AutoTransmission *at, VehicleState *vs, const VehicleSpec *spec,
                              const VehicleDerived *derived, ControllerOutput *io, float dt);

#endif
