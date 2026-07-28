#include "dev_params.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Shorthand: byte offset of a scalar VehicleSpec field. */
#define SPEC_OFFSET(field) offsetof(VehicleSpec, field)

/* Byte offset of one element of a VehicleSpec float array. */
#define SPEC_ARRAY_OFFSET(field, index) \
    (offsetof(VehicleSpec, field) + (size_t)(index) * sizeof(float))

/*
 * The registry.
 *
 * `defaultValue` repeats the config.h constant rather than reading it at runtime, so this
 * table doubles as a readable specification of the stock car. The "params" scenario in
 * tests/physics_tests.c asserts every entry against vehicle_spec_set_default(), so the two
 * cannot silently disagree.
 *
 * Ranges are development limits, not physical claims: they bound what a slider can reach.
 */
static const DevParameter g_params[] = {
    /* ---------------------------------------------------------------------------- body -- */
    { "body.mass", "Body", "kg", SPEC_OFFSET(massKg),
      VEH_MASS_KG, 600.0f, 2500.0f, 10.0f, false,
      "Sprung plus unsprung mass used by every force-to-acceleration conversion." },
    { "body.yaw_inertia", "Body", "kg*m^2", SPEC_OFFSET(yawInertiaKgM2),
      VEH_YAW_INERTIA_KGM2, 500.0f, 5000.0f, 25.0f, false,
      "Yaw moment of inertia about the CG. Lower values make the car rotate more eagerly." },
    { "body.cg_to_front", "Body", "m", SPEC_OFFSET(cgToFrontM),
      VEH_CG_TO_FRONT_M, 0.60f, 2.20f, 0.01f, true,
      "CG to front axle. Sets the front lever arm and the static front load share." },
    { "body.cg_to_rear", "Body", "m", SPEC_OFFSET(cgToRearM),
      VEH_CG_TO_REAR_M, 0.60f, 2.20f, 0.01f, true,
      "CG to rear axle. Wheelbase is always the sum of the two CG distances." },
    { "body.cg_height", "Body", "m", SPEC_OFFSET(cgHeightM),
      VEH_CG_HEIGHT_M, 0.20f, 1.00f, 0.01f, false,
      "CG height above the road. Scales longitudinal load transfer: transfer = m*ax*h/L." },
    { "body.track_front", "Body", "m", SPEC_OFFSET(trackWidthFrontM),
      VEH_TRACK_FRONT_M, 1.00f, 2.20f, 0.01f, true,
      "Front track width; places the two front contact points." },
    { "body.track_rear", "Body", "m", SPEC_OFFSET(trackWidthRearM),
      VEH_TRACK_REAR_M, 1.00f, 2.20f, 0.01f, true,
      "Rear track width; places the two rear contact points." },
    { "body.drag_coefficient", "Body", "", SPEC_OFFSET(dragCoefficient),
      DRAG_COEFFICIENT, 0.10f, 1.20f, 0.01f, false,
      "Dimensionless drag coefficient Cd in 0.5*rho*Cd*A*v^2, opposing the velocity vector." },
    { "body.frontal_area", "Body", "m^2", SPEC_OFFSET(frontalAreaM2),
      FRONTAL_AREA_M2, 1.00f, 3.50f, 0.05f, false,
      "Reference frontal area A paired with the drag coefficient." },
    { "body.rolling_resistance", "Body", "", SPEC_OFFSET(rollingResistanceCoefficient),
      ROLLING_RESISTANCE_COEF, 0.000f, 0.060f, 0.001f, false,
      "Dimensionless rolling resistance coefficient; force is the coefficient times the "
      "wheel's current dynamic normal load, so load transfer moves it front to rear." },
    { "body.load_filter_rate", "Body", "Hz", SPEC_OFFSET(loadFilterRateHz),
      LOAD_FILTER_RATE_HZ, 1.0f, 60.0f, 0.5f, false,
      "Corner frequency of the first-order filter on the previous step's solved longitudinal "
      "acceleration. Lower values make load transfer lag further behind a throttle or brake "
      "change; the filter input is never this step's own acceleration." },
    { "body.roll_stiffness_front", "Body", "", SPEC_OFFSET(rollStiffnessFrontFraction),
      0.50f, 0.00f, 1.00f, 0.01f, false,
      "Front axle share of roll moment m*ay*h. 0.50 is symmetric; higher loads the front "
      "outer wheel more." },

    /* -------------------------------------------------------------------------- wheels -- */
    { "wheel.radius", "Wheels", "m", SPEC_OFFSET(wheelRadiusM),
      WHEEL_RADIUS_M, 0.20f, 0.45f, 0.005f, false,
      "Loaded rolling radius; converts wheel speed to contact-patch speed." },
    { "wheel.inertia", "Wheels", "kg*m^2", SPEC_OFFSET(wheelInertiaKgM2),
      WHEEL_INERTIA_KGM2, 0.30f, 4.00f, 0.05f, false,
      "Rotational inertia of one wheel. Governs how fast a wheel spins up or locks." },

    /* ------------------------------------------------------------------------ steering -- */
    { "steer.max_angle", "Steering", "rad", SPEC_OFFSET(maxRoadWheelAngleRad),
      STEER_MAX_RAD, 0.20f, 1.20f, 0.01f, false,
      "Maximum road-wheel angle at full lock. Left is positive." },
    { "steer.rate", "Steering", "rad/s", SPEC_OFFSET(maxSteerRateRadS),
      STEER_RATE_RAD_S, 0.50f, 20.0f, 0.10f, false,
      "How fast the road wheels follow a steering input." },
    { "steer.return_rate", "Steering", "rad/s", SPEC_OFFSET(steerReturnRateRadS),
      STEER_RETURN_RATE_RAD_S, 0.50f, 25.0f, 0.10f, false,
      "How fast the road wheels centre when the input is released." },
    { "steer.ackermann_percent", "Steering", "", SPEC_OFFSET(ackermannPercent),
      0.00f, 0.00f, 1.00f, 0.01f, false,
      "0=parallel steer, 1=true Ackermann. The inner wheel steers more than the outer." },

    /* --------------------------------------------------------------------------- tires -- */
    { "tire.lat_front.b", "Tires", "", SPEC_OFFSET(tireBLatFront),
      TIRE_B_LAT_FRONT, 2.0f, 25.0f, 0.1f, false,
      "Front lateral stiffness factor: larger reaches peak grip at a smaller slip angle." },
    { "tire.lat_front.c", "Tires", "", SPEC_OFFSET(tireCLatFront),
      TIRE_C_LAT_FRONT, 1.00f, 2.20f, 0.01f, false,
      "Front lateral shape factor: controls how sharply force falls off past the peak." },
    { "tire.lat_front.mu", "Tires", "", SPEC_OFFSET(tireMuLatFront),
      TIRE_MU_LAT_FRONT, 0.30f, 2.00f, 0.01f, false,
      "Front lateral peak friction coefficient, as a multiple of normal load." },
    { "tire.lat_rear.b", "Tires", "", SPEC_OFFSET(tireBLatRear),
      TIRE_B_LAT_REAR, 2.0f, 25.0f, 0.1f, false,
      "Rear lateral stiffness factor." },
    { "tire.lat_rear.c", "Tires", "", SPEC_OFFSET(tireCLatRear),
      TIRE_C_LAT_REAR, 1.00f, 2.20f, 0.01f, false,
      "Rear lateral shape factor." },
    { "tire.lat_rear.mu", "Tires", "", SPEC_OFFSET(tireMuLatRear),
      TIRE_MU_LAT_REAR, 0.30f, 2.00f, 0.01f, false,
      "Rear lateral peak friction. Below the front value the car oversteers sooner." },
    { "tire.long.b", "Tires", "", SPEC_OFFSET(tireBLong),
      TIRE_B_LONG, 2.0f, 30.0f, 0.1f, false,
      "Longitudinal stiffness factor against slip ratio." },
    { "tire.long.c", "Tires", "", SPEC_OFFSET(tireCLong),
      TIRE_C_LONG, 1.00f, 2.20f, 0.01f, false,
      "Longitudinal shape factor." },
    { "tire.long.mu_scale", "Tires", "", SPEC_OFFSET(tireMuLongScale),
      TIRE_MU_LONG_SCALE, 0.30f, 2.00f, 0.01f, false,
      "Longitudinal friction scale applied on top of the lateral peak friction." },
    { "tire.relaxation_length", "Tires", "m", SPEC_OFFSET(tireRelaxationLengthM),
      0.00f, 0.00f, 1.00f, 0.01f, false,
      "First-order lateral-force relaxation length. 0 disables; rate=|vx|/L." },
    { "tire.load_sensitivity_k", "Tires", "", SPEC_OFFSET(tireLoadSensitivityK),
      0.00f, 0.00f, 0.05f, 0.001f, false,
      "Exponent in mu_eff = mu * (Fz/FzRef)^-k. 0 disables." },
    { "tire.load_ref_per_wheel", "Tires", "N", SPEC_OFFSET(tireLoadRefPerWheelN),
      2940.0f, 500.0f, 8000.0f, 10.0f, false,
      "Reference load for the load-sensitivity curve; stock = m*g/4 per wheel." },

    /* ---------------------------------------------------------------------- drivetrain -- */
    { "drive.gear1", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 0),
      3.55f, 0.40f, 6.00f, 0.01f, false, "First gear ratio." },
    { "drive.gear2", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 1),
      2.05f, 0.40f, 6.00f, 0.01f, false, "Second gear ratio." },
    { "drive.gear3", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 2),
      1.38f, 0.40f, 6.00f, 0.01f, false, "Third gear ratio." },
    { "drive.gear4", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 3),
      1.00f, 0.40f, 6.00f, 0.01f, false, "Fourth gear ratio." },
    { "drive.gear5", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 4),
      0.82f, 0.40f, 6.00f, 0.01f, false, "Fifth gear ratio." },
    { "drive.reverse", "Drivetrain", "", SPEC_OFFSET(reverseGearRatio),
      REVERSE_GEAR_RATIO, 0.40f, 6.00f, 0.01f, false, "Reverse gear ratio." },
    { "drive.final", "Drivetrain", "", SPEC_OFFSET(finalDriveRatio),
      FINAL_DRIVE_RATIO, 1.00f, 8.00f, 0.01f, false, "Final drive ratio." },
    { "drive.efficiency", "Drivetrain", "", SPEC_OFFSET(drivetrainEfficiency),
      DRIVETRAIN_EFFICIENCY, 0.50f, 1.00f, 0.01f, false,
      "Fraction of engine torque that reaches the driven wheels." },
    { "engine.idle_rpm", "Drivetrain", "rpm", SPEC_OFFSET(engineIdleRpm),
      ENGINE_IDLE_RPM, 500.0f, 2000.0f, 25.0f, false, "Idle speed floor." },
    { "engine.redline_rpm", "Drivetrain", "rpm", SPEC_OFFSET(engineRedlineRpm),
      ENGINE_REDLINE_RPM, 3000.0f, 10000.0f, 100.0f, false,
      "Redline; also the upper end of the torque curve's rpm axis." },
    { "engine.torque_p0", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 0),
      140.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 0 (idle end)." },
    { "engine.torque_p1", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 1),
      200.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 1." },
    { "engine.torque_p2", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 2),
      240.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 2." },
    { "engine.torque_p3", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 3),
      255.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 3 (peak)." },
    { "engine.torque_p4", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 4),
      250.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 4." },
    { "engine.torque_p5", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 5),
      230.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 5." },
    { "engine.torque_p6", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 6),
      195.0f, 0.0f, 600.0f, 5.0f, false, "Torque curve point 6 (redline end)." },
    { "engine.braking_torque", "Drivetrain", "N*m", SPEC_OFFSET(engineBrakingTorqueNm),
      ENGINE_BRAKING_TORQUE_NM, 0.0f, 200.0f, 1.0f, false,
      "Closed-throttle engine braking torque at the crankshaft." },
    { "drive.diff_mode", "Drivetrain", "", SPEC_OFFSET(differentialMode),
      0.0f, 0.0f, 2.0f, 1.0f, false,
      "0=locked (both rear wheels share omega equally), 1=open (equal torque split), "
      "2=LSD torque-biasing." },
    { "drive.diff_bias_ratio", "Drivetrain", "", SPEC_OFFSET(differentialBiasRatio),
      2.0f, 1.0f, 5.0f, 0.1f, false,
      "LSD: maximum ratio of slower to faster wheel torque." },
    { "drive.diff_preload", "Drivetrain", "N*m", SPEC_OFFSET(differentialPreloadNm),
      60.0f, 0.0f, 400.0f, 5.0f, false,
      "LSD clutch preload torque; minimum torque bias even at zero difference." },

    /* -------------------------------------------------------------------------- brakes -- */
    { "brake.max_torque", "Brakes", "N*m", SPEC_OFFSET(maxBrakeTorqueNm),
      MAX_BRAKE_TORQUE_NM, 0.0f, 8000.0f, 50.0f, false,
      "Total service brake torque at full pedal, before bias." },
    { "brake.bias_front", "Brakes", "", SPEC_OFFSET(brakeBiasFront),
      BRAKE_BIAS_FRONT, 0.0f, 1.0f, 0.01f, false,
      "Fraction of service brake torque sent to the front axle." },
    { "brake.handbrake_torque", "Brakes", "N*m", SPEC_OFFSET(handbrakeTorqueNm),
      HANDBRAKE_TORQUE_NM, 0.0f, 6000.0f, 50.0f, false,
      "Rear-axle handbrake torque at full pull." },

    /* ---------------------------------------------------------------------- collision -- */
    { "collision.half_width", "Collision", "m", SPEC_OFFSET(bodyHalfWidthM),
      VEHICLE_BODY_HALF_WIDTH_M, 0.40f, 1.50f, 0.01f, true,
      "Vehicle body half-width for the collision capsule. Smaller than tyre track width." },
    { "collision.restitution", "Collision", "", SPEC_OFFSET(collisionRestitution),
      COLLISION_RESTITUTION, 0.00f, 0.90f, 0.01f, false,
      "Barrier bounce restitution. 0 = no bounce (full impact absorption); 0.3 = low bounce." },
    { "collision.friction", "Collision", "", SPEC_OFFSET(collisionFriction),
      COLLISION_FRICTION, 0.00f, 1.50f, 0.01f, false,
      "Coulomb friction coefficient at barrier impact. Governs how much a glancing hit spins the car." },
};

#define PARAM_COUNT ((int)(sizeof(g_params) / sizeof(g_params[0])))

/* ------------------------------------------------------------------------------- access -- */

int dev_params_count(void)
{
    return PARAM_COUNT;
}

const DevParameter *dev_params_all(void)
{
    return g_params;
}

const DevParameter *dev_param_at(int index)
{
    if (index < 0 || index >= PARAM_COUNT) return NULL;
    return &g_params[index];
}

const DevParameter *dev_param_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(g_params[i].name, name) == 0) return &g_params[i];
    }
    return NULL;
}

int dev_params_group_count(void)
{
    int count = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        bool seen = false;
        for (int j = 0; j < i; j++) {
            if (strcmp(g_params[i].group, g_params[j].group) == 0) { seen = true; break; }
        }
        if (!seen) count++;
    }
    return count;
}

const char *dev_params_group_name(int groupIndex)
{
    int count = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        bool seen = false;
        for (int j = 0; j < i; j++) {
            if (strcmp(g_params[i].group, g_params[j].group) == 0) { seen = true; break; }
        }
        if (seen) continue;
        if (count == groupIndex) return g_params[i].group;
        count++;
    }
    return NULL;
}

static float *spec_field(VehicleSpec *spec, const DevParameter *param)
{
    return (float *)(void *)((unsigned char *)spec + param->offset);
}

static const float *spec_field_const(const VehicleSpec *spec, const DevParameter *param)
{
    return (const float *)(const void *)((const unsigned char *)spec + param->offset);
}

float dev_param_get(const VehicleSpec *spec, const DevParameter *param)
{
    if (spec == NULL || param == NULL) return 0.0f;
    return *spec_field_const(spec, param);
}

void dev_params_refresh_derived(VehicleSpec *spec)
{
    if (spec == NULL) return;
    /* wheelbase is not independently tunable: vehicle_spec_is_valid() requires it to equal
     * the sum of the two CG distances, so it is recomputed rather than exposed. */
    spec->wheelbaseM = spec->cgToFrontM + spec->cgToRearM;
}

bool dev_param_set(VehicleSpec *spec, const DevParameter *param, float value)
{
    if (spec == NULL || param == NULL) return false;
    if (!isfinite(value)) return false;
    if (value < param->minimum) value = param->minimum;
    if (value > param->maximum) value = param->maximum;
    *spec_field(spec, param) = value;
    dev_params_refresh_derived(spec);
    return true;
}

bool dev_param_is_default(const VehicleSpec *spec, const DevParameter *param)
{
    if (spec == NULL || param == NULL) return true;
    const float value = dev_param_get(spec, param);
    const float span = fmaxf(fabsf(param->defaultValue), 1.0f);
    return fabsf(value - param->defaultValue) <= span * 1e-6f;
}

void dev_param_reset(VehicleSpec *spec, const DevParameter *param)
{
    if (spec == NULL || param == NULL) return;
    *spec_field(spec, param) = param->defaultValue;
    dev_params_refresh_derived(spec);
}

void dev_params_reset_all(VehicleSpec *spec)
{
    if (spec == NULL) return;
    vehicle_spec_set_default(spec);
}

int dev_params_modified_count(const VehicleSpec *spec)
{
    if (spec == NULL) return 0;
    int count = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (!dev_param_is_default(spec, &g_params[i])) count++;
    }
    return count;
}

/* ----------------------------------------------------------------------------- profiles -- */

bool dev_params_save(const VehicleSpec *spec, const char *path)
{
    if (spec == NULL || path == NULL) return false;

    FILE *file = fopen(path, "wb");
    if (file == NULL) return false;

    fprintf(file, "%s\n", DEV_PROFILE_MAGIC);
    fprintf(file, "# name = value    [unit]  (default)\n");

    const char *group = NULL;
    for (int i = 0; i < PARAM_COUNT; i++) {
        const DevParameter *param = &g_params[i];
        if (group == NULL || strcmp(group, param->group) != 0) {
            group = param->group;
            fprintf(file, "\n# --- %s ---\n", group);
        }
        fprintf(file, "%-24s = %-12.6f # %s%s(default %.6f)\n",
                param->name, (double)dev_param_get(spec, param),
                param->unit[0] != '\0' ? param->unit : "",
                param->unit[0] != '\0' ? " " : "",
                (double)param->defaultValue);
    }

    const bool ok = (fflush(file) == 0) && !ferror(file);
    return (fclose(file) == 0) && ok;
}

/* Parse one `name = value` line. Returns 0 on a blank/comment line, 1 when a key/value pair
 * was extracted, -1 when the line is malformed. */
static int parse_line(const char *line, size_t length, char *nameOut, size_t nameCap,
                      float *valueOut)
{
    size_t i = 0;
    while (i < length && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= length || line[i] == '#' || line[i] == ';' || line[i] == '\r') return 0;

    const size_t nameStart = i;
    while (i < length && line[i] != '=' && line[i] != ' ' && line[i] != '\t') i++;
    const size_t nameLength = i - nameStart;
    if (nameLength == 0 || nameLength >= nameCap) return -1;

    while (i < length && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= length || line[i] != '=') return -1;
    i++;
    while (i < length && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= length) return -1;

    /* strtof needs a NUL-terminated buffer; copy the remainder of the line. */
    char valueBuffer[64];
    size_t valueLength = 0;
    while (i < length && valueLength + 1 < sizeof(valueBuffer) &&
           line[i] != '#' && line[i] != ';' && line[i] != '\r' && line[i] != '\n') {
        valueBuffer[valueLength++] = line[i++];
    }
    valueBuffer[valueLength] = '\0';

    char *end = NULL;
    const double parsed = strtod(valueBuffer, &end);
    if (end == valueBuffer) return -1;
    while (end != NULL && *end != '\0' && (*end == ' ' || *end == '\t')) end++;
    if (end != NULL && *end != '\0') return -1;
    if (!isfinite(parsed)) return -1;

    memcpy(nameOut, line + nameStart, nameLength);
    nameOut[nameLength] = '\0';
    *valueOut = (float)parsed;
    return 1;
}

bool dev_params_apply_text(VehicleSpec *spec, const char *text, size_t length,
                           int *appliedOut, int *unknownOut, int *rejectedOut)
{
    int applied = 0;
    int unknown = 0;
    int rejected = 0;

    if (appliedOut != NULL)  *appliedOut = 0;
    if (unknownOut != NULL)  *unknownOut = 0;
    if (rejectedOut != NULL) *rejectedOut = 0;
    if (spec == NULL || text == NULL) return false;

    /* Work on a copy: a profile that produces an invalid spec must change nothing. */
    VehicleSpec candidate = *spec;

    size_t offset = 0;
    while (offset < length) {
        size_t end = offset;
        while (end < length && text[end] != '\n') end++;

        char name[64];
        float value = 0.0f;
        const int status = parse_line(text + offset, end - offset, name, sizeof(name), &value);
        if (status == 1) {
            const DevParameter *param = dev_param_find(name);
            if (param == NULL) {
                unknown++;
            } else if (dev_param_set(&candidate, param, value)) {
                applied++;
            } else {
                rejected++;
            }
        } else if (status < 0) {
            rejected++;
        }

        offset = (end < length) ? end + 1 : length;
    }

    dev_params_refresh_derived(&candidate);
    if (!vehicle_spec_is_valid(&candidate)) return false;

    *spec = candidate;
    if (appliedOut != NULL)  *appliedOut = applied;
    if (unknownOut != NULL)  *unknownOut = unknown;
    if (rejectedOut != NULL) *rejectedOut = rejected;
    return true;
}

bool dev_params_load(VehicleSpec *spec, const char *path,
                     int *appliedOut, int *unknownOut, int *rejectedOut)
{
    if (spec == NULL || path == NULL) return false;

    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;

    /* Profiles are small by construction; refuse anything absurd rather than allocating it. */
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return false; }
    const long size = ftell(file);
    if (size < 0 || size > (1L << 20)) { fclose(file); return false; }
    if (fseek(file, 0, SEEK_SET) != 0) { fclose(file); return false; }

    char *buffer = (char *)malloc((size_t)size + 1u);
    if (buffer == NULL) { fclose(file); return false; }

    const size_t read = fread(buffer, 1, (size_t)size, file);
    fclose(file);
    buffer[read] = '\0';

    const bool ok = dev_params_apply_text(spec, buffer, read,
                                          appliedOut, unknownOut, rejectedOut);
    free(buffer);
    return ok;
}

/* ------------------------------------------------------------------------------ reports -- */

void dev_params_write_markdown(FILE *out)
{
    if (out == NULL) return;

    fprintf(out, "<!-- Generated by `drifty_tests --dump-params`. Do not edit by hand. -->\n");
    fprintf(out, "# Tunable parameters\n\n");
    fprintf(out, "Every row is one entry in the `src/dev_params.c` registry, which is also what\n");
    fprintf(out, "generates the Physics Lab sliders and the tuning-profile format.\n");

    const int groups = dev_params_group_count();
    for (int g = 0; g < groups; g++) {
        const char *group = dev_params_group_name(g);
        fprintf(out, "\n## %s\n\n", group);
        fprintf(out, "| Parameter | Default | Unit | Range | Step | Reset | Meaning |\n");
        fprintf(out, "|---|---:|---|---|---:|:-:|---|\n");
        for (int i = 0; i < PARAM_COUNT; i++) {
            const DevParameter *param = &g_params[i];
            if (strcmp(param->group, group) != 0) continue;
            fprintf(out, "| `%s` | %g | %s | %g .. %g | %g | %s | %s |\n",
                    param->name, (double)param->defaultValue,
                    param->unit[0] != '\0' ? param->unit : "—",
                    (double)param->minimum, (double)param->maximum, (double)param->step,
                    param->requiresRestart ? "yes" : "—",
                    param->description);
        }
    }

    fprintf(out, "\n`Reset` marks a parameter that only takes full effect after a simulation\n");
    fprintf(out, "reset, because it moves the wheel contact points.\n");
}

void dev_params_write_metadata(FILE *out, const VehicleSpec *spec)
{
    if (out == NULL || spec == NULL) return;
    fprintf(out, "# parameter,value,default,unit\n");
    for (int i = 0; i < PARAM_COUNT; i++) {
        const DevParameter *param = &g_params[i];
        fprintf(out, "# %s,%.6f,%.6f,%s\n", param->name,
                (double)dev_param_get(spec, param), (double)param->defaultValue,
                param->unit[0] != '\0' ? param->unit : "-");
    }
}

int dev_params_write_overrides(FILE *out, const VehicleSpec *spec)
{
    if (out == NULL || spec == NULL) return 0;
    int written = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        const DevParameter *param = &g_params[i];
        if (dev_param_is_default(spec, param)) continue;
        fprintf(out, "%s%s=%.6f", (written > 0) ? " " : "", param->name,
                (double)dev_param_get(spec, param));
        written++;
    }
    return written;
}
