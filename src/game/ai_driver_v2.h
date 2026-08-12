/*
 * ai_driver_v2.h — the #79 limit-aware driver (AI_DRIVER_ARCH_LIMIT).
 *
 * Same signature and same audit boundary as the baseline driver: every input is const, the
 * only writes are the ControllerOutput and the driver's own scratch state, and the plan it
 * drives is the same shared minimum-curvature lattice. See ai_driver_v2.c for the design.
 */
#ifndef CIRCUIT_AI_DRIVER_V2_H
#define CIRCUIT_AI_DRIVER_V2_H

#include "game/ai_driver.h"

/* One tick of driving with the limit-aware architecture. Replaces ai_driver_update()'s body
 * when AiDriverConfig.architecture == AI_DRIVER_ARCH_LIMIT. */
void ai_driver_update_v2(const AiDriverConfig *cfg, AiDriverState *state,
                         const TrackDefinition *track, const TrackRuntime *runtime,
                         const VehicleState *vehicle, const VehicleDerived *derived,
                         const VehicleSpec *spec, ControllerOutput *out, float dt,
                         const AiTraffic *traffic);

#endif /* CIRCUIT_AI_DRIVER_V2_H */
