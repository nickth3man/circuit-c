/*
 * auto_transmission.h — automatic gear selection and arcade direction-swap state machine.
 */
#ifndef CIRCUIT_AUTO_TRANSMISSION_H
#define CIRCUIT_AUTO_TRANSMISSION_H

#include <stdbool.h>

#include "physics/vehicle.h"
#include "game/input.h"

void auto_transmission_update(AutoTransmission *at, VehicleState *vs, const VehicleSpec *spec,
                              const VehicleDerived *derived, Input *io, float dt);

#endif
