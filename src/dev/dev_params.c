#include "dev/dev_params.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SPEC_OFFSET(field) offsetof(VehicleSpec, field)
#define SPEC_ARRAY_OFFSET(field, index) \
    (offsetof(VehicleSpec, field) + (size_t)(index) * sizeof(float))

/*
 * The registry.
 *
 * `defaultValue` repeats the config.h constant rather than reading it at runtime, so this
 * table doubles as a readable specification of the stock car. The "params" scenario in
 * tests/scenarios/physics_tests.c asserts every entry against vehicle_spec_set_default(), so the two
 * cannot silently disagree.
 *
 * Ranges are development limits, not physical claims: they bound what a slider can reach.
 * Phase 2 spans kei-car to two-axle commercial examples.
 */
static const DevParameter g_params[] = {
    { "body.wheelbase", "Body", "m", SPEC_OFFSET(wheelbaseM), VEH_WHEELBASE_M, 1.8f, 7.0f,
      0.01f, true, false, 0,
      "Wheelbase; primary. CG distances derive from this plus mass particles." },
    { "body.track_front", "Body", "m", SPEC_OFFSET(trackWidthFrontM), VEH_TRACK_FRONT_M, 1.0f,
      2.6f, 0.01f, true, false, 0, "Front track width." },
    { "body.track_rear", "Body", "m", SPEC_OFFSET(trackWidthRearM), VEH_TRACK_REAR_M, 1.0f,
      2.6f, 0.01f, true, false, 0, "Rear track width." },
    { "body.front_overhang", "Body", "m", SPEC_OFFSET(frontOverhangM), VEH_FRONT_OVERHANG_M,
      0.2f, 2.5f, 0.01f, false, false, 1, "Body ahead of the front axle." },
    { "body.rear_overhang", "Body", "m", SPEC_OFFSET(rearOverhangM), VEH_REAR_OVERHANG_M, 0.2f,
      2.5f, 0.01f, false, false, 1, "Body behind the rear axle." },
    { "body.width_overall", "Body", "m", SPEC_OFFSET(widthOverallM), VEH_WIDTH_OVERALL_M, 1.2f,
      2.6f, 0.01f, true, false, 0, "Overall body width; derives collision half-width." },
    { "body.height_overall", "Body", "m", SPEC_OFFSET(heightOverallM), VEH_HEIGHT_OVERALL_M,
      1.0f, 3.2f, 0.01f, false, false, 0,
      "Overall body height; derives frontal area with width." },
    { "body.ride_height_front", "Body", "m", SPEC_OFFSET(rideHeightFrontM),
      VEH_RIDE_HEIGHT_FRONT_M, 0.04f, 0.5f, 0.005f, false, false, 1,
      "Front ride height / arch clearance." },
    { "body.ride_height_rear", "Body", "m", SPEC_OFFSET(rideHeightRearM),
      VEH_RIDE_HEIGHT_REAR_M, 0.04f, 0.5f, 0.005f, false, false, 1,
      "Rear ride height / arch clearance." },
    { "body.cowl_x", "Body", "m", SPEC_OFFSET(cowlXM), VEH_COWL_X_M, -2.0f, 2.0f, 0.01f, false,
      false, 1, "Cowl X in layout frame (axle midpoint origin)." },
    { "body.backlight_x", "Body", "m", SPEC_OFFSET(backlightXM), VEH_BACKLIGHT_X_M, -2.0f, 2.0f,
      0.01f, false, false, 1, "Backlight X in layout frame." },
    /* Maximum is 1.5 m, not the ~2.0 m of a real long bed, because this is a sweep axis and
     * car_visual.c clamps the bed to the space behind the greenhouse — about 1.57 m on the
     * stock car. A range reaching past that would make the top sweep steps clamp to the same
     * value and collapse into each other. Ranges here are development limits, not physical
     * claims; a longer bed on a longer truck is a corpus question, not a registry one. */
    { "body.bed_length", "Body", "m", SPEC_OFFSET(bedLengthM), VEH_BED_LENGTH_M, 0.0f, 1.5f,
      0.05f, false, false, 1,
      "Open cargo bed forward from the tail; 0 = none. Declared, never inferred." },
    /* Phase B silhouette control. nose_width and tail_width are [identity] hull endpoints:
     * the half-width the rasterizer draws at the foremost/rearmost station is exactly half
     * this value. shoulder_x is the layout-frame station of maximum width (widthOverallM).
     * Ranges are development limits, not physical claims; the px effect at 13.2 px/m is stated
     * in each entry, and validity rejects an endpoint wider than widthOverallM (see vehicle.c). */
    { "body.nose_width", "Body", "m", SPEC_OFFSET(noseWidthM), VEH_NOSE_WIDTH_M, 0.8f, 2.2f,
      0.01f, false, false, 1,
      "Width at the foremost hull station (full width). [identity] endpoint; 0.8..2.2 m = "
      "11..29 px." },
    { "body.tail_width", "Body", "m", SPEC_OFFSET(tailWidthM), VEH_TAIL_WIDTH_M, 0.8f, 2.4f,
      0.01f, false, false, 1,
      "Width at the rearmost hull station (full width). [identity] endpoint; 0.8..2.4 m = "
      "11..32 px." },
    { "body.shoulder_x", "Body", "m", SPEC_OFFSET(shoulderXM), VEH_SHOULDER_X_M, -2.0f, 2.0f,
      0.01f, false, false, 1,
      "Station of maximum body width, layout frame (axle midpoint origin). Same convention as "
      "cowl_x/backlight_x; moves the widest point over ~53 px of body length at stock." },
    { "body.fender_flare_front", "Body", "m", SPEC_OFFSET(fenderFlareFrontM),
      VEH_FENDER_FLARE_FRONT_M, 0.0f, 0.12f, 0.002f, false, false, 1,
      "Front arch flare proud of the hull over the wheel arch; 0 = none. 0..0.12 m = 0..1.6 "
      "px." },
    { "body.fender_flare_rear", "Body", "m", SPEC_OFFSET(fenderFlareRearM),
      VEH_FENDER_FLARE_REAR_M, 0.0f, 0.15f, 0.002f, false, false, 1,
      "Rear arch flare proud of the hull over the wheel arch; 0 = none. 0..0.15 m = 0..2 px." },
    /* Phase C greenhouse. Longitudinal stations remain in the layout frame. Rake angles are
     * SI radians; comments state the degree envelope used by vehicle measurements. */
    { "body.roof_start_x", "Body", "m", SPEC_OFFSET(roofStartXM), VEH_ROOF_START_X_M, -4.0f,
      4.0f, 0.01f, false, false, 1,
      "Forward roof edge, layout frame; -4..4 m moves over 106 px." },
    { "body.roof_end_x", "Body", "m", SPEC_OFFSET(roofEndXM), VEH_ROOF_END_X_M, -4.0f, 4.0f,
      0.01f, false, false, 1, "Aft roof edge, layout frame; -4..4 m moves over 106 px." },
    { "body.roof_width", "Body", "m", SPEC_OFFSET(roofWidthM), VEH_ROOF_WIDTH_M, 0.3f, 2.3f,
      0.01f, false, false, 1,
      "Full physical roof width; 0.3..2.3 m = 4..30 px. Tumblehome inset is "
      "(body width - roof width)/2." },
    { "body.windscreen_rake", "Body", "rad", SPEC_OFFSET(windscreenRakeRad),
      VEH_WINDSCREEN_RAKE_RAD, VEH_WINDSCREEN_RAKE_MIN_RAD, VEH_WINDSCREEN_RAKE_MAX_RAD, 0.01f,
      false, false, 1,
      "A-pillar angle from vertical, 20..70 deg; moves glass by ~7 px and feeds effective "
      "Cd." },
    { "body.backlight_rake", "Body", "rad", SPEC_OFFSET(backlightRakeRad),
      VEH_BACKLIGHT_RAKE_RAD, VEH_BACKLIGHT_RAKE_MIN_RAD, VEH_BACKLIGHT_RAKE_MAX_RAD, 0.01f,
      false, false, 1, "Rear-screen angle from vertical, 15..75 deg; moves glass by ~6 px." },
    { "body.side_window_count", "Body", "", SPEC_OFFSET(sideWindowCount), VEH_SIDE_WINDOW_COUNT,
      2.0f, 6.0f, 1.0f, false, false, 1,
      "Side-glass segment count; exact integer 2..6, each segment about 8..10 px." },
    { "body.quarter_window", "Body", "m", SPEC_OFFSET(quarterWindowLengthM),
      VEH_QUARTER_WINDOW_LENGTH_M, 0.0f, 0.4f, 0.02f, false, false, 1,
      "Quarter glass aft of the rear door; 0 = none, otherwise 0.2..0.4 m = 3..5 px." },
    { "body.sunroof_length", "Body", "m", SPEC_OFFSET(sunroofLengthM), VEH_SUNROOF_LENGTH_M,
      0.0f, 1.0f, 0.05f, false, false, 1,
      "Roof glass length; 0 = none, otherwise 0.4..1.0 m = 5..13 px." },
    { "body.door_count", "Body", "", SPEC_OFFSET(doorCount), VEH_DOOR_COUNT, 2.0f, 5.0f, 1.0f,
      false, false, 1, "Door count; exact domain 2, 4, or 5; drives visible pillars." },
    { "body.cabin_rows", "Body", "", SPEC_OFFSET(cabinRows), VEH_CABIN_ROWS, 1.0f, 3.0f, 1.0f,
      false, false, 1, "Seat rows; exact integer 1..3, packaged rearward from mass.driver_x." },
    { "body.roof_type", "Body", "", SPEC_OFFSET(roofType), VEH_ROOF_TYPE, 0.0f, 2.0f, 1.0f,
      false, false, 1, "Roof enum: 0=fixed, 1=targa, 2=convertible." },
    { "body.drag_coefficient", "Body", "", SPEC_OFFSET(dragCoefficient), DRAG_COEFFICIENT, 0.1f,
      1.2f, 0.01f, false, false, 1, "Cd in 0.5*rho*Cd*A*v^2." },
    { "body.load_filter_rate", "Body", "Hz", SPEC_OFFSET(loadFilterRateHz), LOAD_FILTER_RATE_HZ,
      1.0f, 60.0f, 0.5f, false, false, 2, "Load-transfer accel filter corner frequency." },
    { "body.roll_stiffness_front", "Body", "", SPEC_OFFSET(rollStiffnessFrontFraction),
      ROLL_STIFFNESS_FRONT_FRACTION, 0.0f, 1.0f, 0.01f, false, false, 1,
      "Front axle share of roll moment." },
    { "body.mass", "Body", "kg", SPEC_OFFSET(massKg), VEH_MASS_KG, 400.0f, 15000.0f, 10.0f,
      false, true, 0, "Total mass from particles (read-only)." },
    { "body.yaw_inertia", "Body", "kg*m^2", SPEC_OFFSET(yawInertiaKgM2), VEH_YAW_INERTIA_KGM2,
      200.0f, 40000.0f, 25.0f, false, true, 1, "Yaw inertia from particles (read-only)." },
    { "body.cg_to_front", "Body", "m", SPEC_OFFSET(cgToFrontM), VEH_CG_TO_FRONT_M, 0.4f, 4.0f,
      0.01f, true, true, 0, "CG to front axle (derived)." },
    { "body.cg_to_rear", "Body", "m", SPEC_OFFSET(cgToRearM), VEH_CG_TO_REAR_M, 0.4f, 4.0f,
      0.01f, true, true, 0, "CG to rear axle (derived)." },
    { "body.cg_height", "Body", "m", SPEC_OFFSET(cgHeightM), VEH_CG_HEIGHT_M, 0.15f, 1.5f,
      0.01f, false, true, 1, "CG height (derived)." },
    { "body.frontal_area", "Body", "m^2", SPEC_OFFSET(frontalAreaM2),
      VEH_WIDTH_OVERALL_M * VEH_HEIGHT_OVERALL_M * VEH_FRONTAL_AREA_FILL, 1.0f, 8.0f, 0.05f,
      false, true, 1, "Frontal area from width*height*fill (derived)." },
    { "body.length_overall", "Body", "m", SPEC_OFFSET(lengthOverallM), VEH_LENGTH_OVERALL_M,
      2.5f, 12.0f, 0.01f, false, true, 1, "Overall length = wheelbase + overhangs." },
    { "mass.engine", "Mass", "kg", SPEC_OFFSET(massEngineKg), MASS_ENGINE_KG, 20.0f, 8000.0f,
      5.0f, false, false, 0, "Engine mass particle." },
    { "mass.engine_x", "Mass", "m", SPEC_OFFSET(massEngineXM), MASS_ENGINE_X_M, -4.0f, 4.0f,
      0.01f, false, false, 0, "Engine X in layout frame." },
    { "mass.engine_z", "Mass", "m", SPEC_OFFSET(massEngineZM), MASS_ENGINE_Z_M, 0.05f, 2.0f,
      0.01f, false, false, 2, "Engine Z (height) in layout frame." },
    { "mass.gearbox", "Mass", "kg", SPEC_OFFSET(massGearboxKg), MASS_GEARBOX_KG, 20.0f, 8000.0f,
      5.0f, false, false, 1, "Gearbox mass particle." },
    { "mass.gearbox_x", "Mass", "m", SPEC_OFFSET(massGearboxXM), MASS_GEARBOX_X_M, -4.0f, 4.0f,
      0.01f, false, false, 1, "Gearbox X in layout frame." },
    { "mass.gearbox_z", "Mass", "m", SPEC_OFFSET(massGearboxZM), MASS_GEARBOX_Z_M, 0.05f, 2.0f,
      0.01f, false, false, 2, "Gearbox Z (height) in layout frame." },
    { "mass.fuel", "Mass", "kg", SPEC_OFFSET(massFuelKg), MASS_FUEL_KG, 20.0f, 8000.0f, 5.0f,
      false, false, 1, "Fuel mass particle." },
    { "mass.fuel_x", "Mass", "m", SPEC_OFFSET(massFuelXM), MASS_FUEL_X_M, -4.0f, 4.0f, 0.01f,
      false, false, 1, "Fuel X in layout frame." },
    { "mass.fuel_z", "Mass", "m", SPEC_OFFSET(massFuelZM), MASS_FUEL_Z_M, 0.05f, 2.0f, 0.01f,
      false, false, 2, "Fuel Z (height) in layout frame." },
    { "mass.driver", "Mass", "kg", SPEC_OFFSET(massDriverKg), MASS_DRIVER_KG, 20.0f, 8000.0f,
      5.0f, false, false, 1, "Driver mass particle." },
    { "mass.driver_x", "Mass", "m", SPEC_OFFSET(massDriverXM), MASS_DRIVER_X_M, -4.0f, 4.0f,
      0.01f, false, false, 1, "Driver X in layout frame." },
    { "mass.driver_z", "Mass", "m", SPEC_OFFSET(massDriverZM), MASS_DRIVER_Z_M, 0.05f, 2.0f,
      0.01f, false, false, 2, "Driver Z (height) in layout frame." },
    { "mass.chassis", "Mass", "kg", SPEC_OFFSET(massChassisKg), MASS_CHASSIS_KG, 20.0f, 8000.0f,
      5.0f, false, false, 0, "Chassis mass particle." },
    { "mass.chassis_x", "Mass", "m", SPEC_OFFSET(massChassisXM), MASS_CHASSIS_X_M, -4.0f, 4.0f,
      0.01f, false, false, 1, "Chassis X in layout frame." },
    { "mass.chassis_z", "Mass", "m", SPEC_OFFSET(massChassisZM), MASS_CHASSIS_Z_M, 0.05f, 2.0f,
      0.01f, false, false, 2, "Chassis Z (height) in layout frame." },
    { "wheel.inertia", "Wheels", "kg*m^2", SPEC_OFFSET(wheelInertiaKgM2), WHEEL_INERTIA_KGM2,
      0.3f, 8.0f, 0.05f, false, false, 1, "Rotational inertia of one wheel." },
    { "wheel.radius", "Wheels", "m", SPEC_OFFSET(wheelRadiusM), WHEEL_RADIUS_M, 0.15f, 0.6f,
      0.005f, false, true, 1, "Legacy rear rolling radius (derived)." },
    { "wheel.radius_front", "Wheels", "m", SPEC_OFFSET(wheelRadiusFrontM), WHEEL_RADIUS_M,
      0.15f, 0.6f, 0.005f, false, true, 0, "Front loaded rolling radius (derived)." },
    { "wheel.radius_rear", "Wheels", "m", SPEC_OFFSET(wheelRadiusRearM), WHEEL_RADIUS_M, 0.15f,
      0.6f, 0.005f, false, true, 0, "Rear loaded rolling radius (derived)." },
    { "wheel.offset_et_front", "Wheels", "mm", SPEC_OFFSET(wheelOffsetEtFrontMm),
      WHEEL_OFFSET_ET_FRONT_MM, -20.0f, 60.0f, 1.0f, false, false, 2,
      "Front wheel ET offset." },
    { "wheel.offset_et_rear", "Wheels", "mm", SPEC_OFFSET(wheelOffsetEtRearMm),
      WHEEL_OFFSET_ET_REAR_MM, -20.0f, 60.0f, 1.0f, false, false, 2, "Rear wheel ET offset." },
    { "tire.section_width_front", "Tires", "mm", SPEC_OFFSET(tireSectionWidthFrontMm),
      TIRE_SECTION_WIDTH_MM, 145.0f, 355.0f, 5.0f, false, false, 0,
      "Front tire section width." },
    { "tire.aspect_front", "Tires", "%", SPEC_OFFSET(tireAspectFrontPct), TIRE_ASPECT_RATIO_PCT,
      25.0f, 80.0f, 1.0f, false, false, 1, "Front tire aspect ratio." },
    { "tire.rim_diameter_front", "Tires", "in", SPEC_OFFSET(tireRimDiameterFrontIn),
      TIRE_RIM_DIAMETER_IN, 12.0f, 22.0f, 0.5f, false, false, 1, "Front rim diameter." },
    { "tire.rim_width_front", "Tires", "in", SPEC_OFFSET(tireRimWidthFrontIn),
      TIRE_RIM_WIDTH_IN, 4.0f, 14.0f, 0.5f, false, false, 2, "Front rim width." },
    { "tire.pressure_front", "Tires", "kPa", SPEC_OFFSET(tirePressureFrontKpa),
      TIRE_PRESSURE_KPA, 120.0f, 400.0f, 5.0f, false, false, 2, "Front cold pressure." },
    { "tire.section_width_rear", "Tires", "mm", SPEC_OFFSET(tireSectionWidthRearMm),
      TIRE_SECTION_WIDTH_MM, 145.0f, 355.0f, 5.0f, false, false, 0,
      "Rear tire section width." },
    { "tire.aspect_rear", "Tires", "%", SPEC_OFFSET(tireAspectRearPct), TIRE_ASPECT_RATIO_PCT,
      25.0f, 80.0f, 1.0f, false, false, 1, "Rear tire aspect ratio." },
    { "tire.rim_diameter_rear", "Tires", "in", SPEC_OFFSET(tireRimDiameterRearIn),
      TIRE_RIM_DIAMETER_IN, 12.0f, 22.0f, 0.5f, false, false, 1, "Rear rim diameter." },
    { "tire.rim_width_rear", "Tires", "in", SPEC_OFFSET(tireRimWidthRearIn), TIRE_RIM_WIDTH_IN,
      4.0f, 14.0f, 0.5f, false, false, 2, "Rear rim width." },
    { "tire.pressure_rear", "Tires", "kPa", SPEC_OFFSET(tirePressureRearKpa), TIRE_PRESSURE_KPA,
      120.0f, 400.0f, 5.0f, false, false, 2, "Rear cold pressure." },
    { "tire.lat_front.b", "Tires", "", SPEC_OFFSET(tireBLatFront), TIRE_B_LAT_FRONT, 2.0f,
      25.0f, 0.1f, false, false, 1, "Front lateral stiffness factor." },
    { "tire.lat_front.c", "Tires", "", SPEC_OFFSET(tireCLatFront), TIRE_C_LAT_FRONT, 1.0f, 2.2f,
      0.01f, false, false, 1, "Front lateral shape factor." },
    { "tire.lat_front.mu", "Tires", "", SPEC_OFFSET(tireMuLatFront), TIRE_MU_LAT_FRONT, 0.3f,
      2.0f, 0.01f, false, false, 1, "Front lateral peak friction." },
    { "tire.lat_rear.b", "Tires", "", SPEC_OFFSET(tireBLatRear), TIRE_B_LAT_REAR, 2.0f, 25.0f,
      0.1f, false, false, 1, "Rear lateral stiffness factor." },
    { "tire.lat_rear.c", "Tires", "", SPEC_OFFSET(tireCLatRear), TIRE_C_LAT_REAR, 1.0f, 2.2f,
      0.01f, false, false, 1, "Rear lateral shape factor." },
    { "tire.lat_rear.mu", "Tires", "", SPEC_OFFSET(tireMuLatRear), TIRE_MU_LAT_REAR, 0.3f, 2.0f,
      0.01f, false, false, 1, "Rear lateral peak friction." },
    { "tire.long.b", "Tires", "", SPEC_OFFSET(tireBLong), TIRE_B_LONG, 2.0f, 30.0f, 0.1f, false,
      false, 1, "Longitudinal stiffness factor." },
    { "tire.long.c", "Tires", "", SPEC_OFFSET(tireCLong), TIRE_C_LONG, 1.0f, 2.2f, 0.01f, false,
      false, 1, "Longitudinal shape factor." },
    { "tire.long.mu_scale", "Tires", "", SPEC_OFFSET(tireMuLongScale), TIRE_MU_LONG_SCALE, 0.3f,
      2.0f, 0.01f, false, false, 1, "Longitudinal friction scale." },
    { "tire.relaxation_length", "Tires", "m", SPEC_OFFSET(tireRelaxationLengthM),
      TIRE_RELAXATION_LENGTH_M, 0.0f, 1.0f, 0.01f, false, false, 2,
      "Lateral force relaxation length." },
    { "tire.load_sensitivity_k", "Tires", "", SPEC_OFFSET(tireLoadSensitivityK),
      TIRE_LOAD_SENSITIVITY_K, 0.0f, 0.05f, 0.001f, false, false, 2,
      "Load sensitivity exponent." },
    { "tire.load_ref_per_wheel", "Tires", "N", SPEC_OFFSET(tireLoadRefPerWheelN),
      TIRE_LOAD_REF_PER_WHEEL_N, 500.0f, 15000.0f, 10.0f, false, true, 2,
      "Reference load = m*g/4 (derived)." },
    { "steer.max_angle", "Steering", "rad", SPEC_OFFSET(maxRoadWheelAngleRad), STEER_MAX_RAD,
      0.2f, 1.2f, 0.01f, false, false, 0, "Max road-wheel angle." },
    { "steer.rate", "Steering", "rad/s", SPEC_OFFSET(maxSteerRateRadS), STEER_RATE_RAD_S, 0.5f,
      20.0f, 0.1f, false, false, 1, "Steer follow rate." },
    { "steer.return_rate", "Steering", "rad/s", SPEC_OFFSET(steerReturnRateRadS),
      STEER_RETURN_RATE_RAD_S, 0.5f, 25.0f, 0.1f, false, false, 1, "Steer return rate." },
    { "steer.speed_ref", "Steering", "m/s", SPEC_OFFSET(steerSpeedRefMps), STEER_SPEED_REF_MPS,
      1.0f, 30.0f, 0.5f, false, false, 1, "Speed below which steering is at full rate." },
    { "steer.speed_min_factor", "Steering", "", SPEC_OFFSET(steerSpeedMinFactor),
      STEER_SPEED_MIN_FACTOR, 0.05f, 1.0f, 0.01f, false, false, 2,
      "Minimum steering rate fraction at high speed." },
    { "steer.ackermann_percent", "Steering", "", SPEC_OFFSET(ackermannPercent),
      ACKERMANN_PERCENT, 0.0f, 1.0f, 0.01f, false, false, 1, "Ackermann blend." },
    { "susp.camber_front", "Suspension", "rad", SPEC_OFFSET(suspCamberFrontRad),
      SUSP_CAMBER_FRONT_RAD, -0.12f, 0.05f, 0.001f, false, false, 2, "Front static camber." },
    { "susp.camber_rear", "Suspension", "rad", SPEC_OFFSET(suspCamberRearRad),
      SUSP_CAMBER_REAR_RAD, -0.12f, 0.05f, 0.001f, false, false, 2, "Rear static camber." },
    { "susp.toe_front", "Suspension", "rad", SPEC_OFFSET(suspToeFrontRad), SUSP_TOE_FRONT_RAD,
      -0.05f, 0.05f, 0.001f, false, false, 2, "Front static toe." },
    { "susp.toe_rear", "Suspension", "rad", SPEC_OFFSET(suspToeRearRad), SUSP_TOE_REAR_RAD,
      -0.05f, 0.05f, 0.001f, false, false, 2, "Rear static toe." },
    { "susp.caster_front", "Suspension", "rad", SPEC_OFFSET(suspCasterFrontRad),
      SUSP_CASTER_FRONT_RAD, 0.0f, 0.25f, 0.001f, false, false, 2, "Front caster." },
    { "susp.caster_rear", "Suspension", "rad", SPEC_OFFSET(suspCasterRearRad),
      SUSP_CASTER_REAR_RAD, 0.0f, 0.25f, 0.001f, false, false, 2, "Rear caster." },
    { "susp.wheel_rate_front", "Suspension", "N/m", SPEC_OFFSET(suspWheelRateFrontNpm),
      SUSP_WHEEL_RATE_FRONT_NPM, 5000.0f, 80000.0f, 100.0f, false, false, 2,
      "Front wheel rate." },
    { "susp.wheel_rate_rear", "Suspension", "N/m", SPEC_OFFSET(suspWheelRateRearNpm),
      SUSP_WHEEL_RATE_REAR_NPM, 5000.0f, 80000.0f, 100.0f, false, false, 2,
      "Rear wheel rate." },
    { "susp.anti_roll_front", "Suspension", "N/m", SPEC_OFFSET(suspAntiRollFrontNpm),
      SUSP_ANTI_ROLL_FRONT_NPM, 0.0f, 60000.0f, 100.0f, false, false, 2,
      "Front anti-roll stiffness." },
    { "susp.anti_roll_rear", "Suspension", "N/m", SPEC_OFFSET(suspAntiRollRearNpm),
      SUSP_ANTI_ROLL_REAR_NPM, 0.0f, 60000.0f, 100.0f, false, false, 2,
      "Rear anti-roll stiffness." },
    { "susp.travel_front", "Suspension", "m", SPEC_OFFSET(suspTravelFrontM),
      SUSP_TRAVEL_FRONT_M, 0.03f, 0.25f, 0.005f, false, false, 1, "Front suspension travel." },
    { "susp.travel_rear", "Suspension", "m", SPEC_OFFSET(suspTravelRearM), SUSP_TRAVEL_REAR_M,
      0.03f, 0.25f, 0.005f, false, false, 1, "Rear suspension travel." },
    { "susp.roll_centre_front", "Suspension", "m", SPEC_OFFSET(suspRollCentreFrontM),
      SUSP_ROLL_CENTRE_FRONT_M, 0.0f, 0.4f, 0.005f, false, false, 2,
      "Front roll-centre height." },
    { "susp.roll_centre_rear", "Suspension", "m", SPEC_OFFSET(suspRollCentreRearM),
      SUSP_ROLL_CENTRE_REAR_M, 0.0f, 0.4f, 0.005f, false, false, 2,
      "Rear roll-centre height." },
    { "drive.gear1", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 0), 3.55f, 0.4f, 6.0f,
      0.01f, false, false, 1, "First gear ratio." },
    { "drive.gear2", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 1), 2.05f, 0.4f, 6.0f,
      0.01f, false, false, 1, "Second gear ratio." },
    { "drive.gear3", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 2), 1.38f, 0.4f, 6.0f,
      0.01f, false, false, 1, "Third gear ratio." },
    { "drive.gear4", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 3), 1.00f, 0.4f, 6.0f,
      0.01f, false, false, 1, "Fourth gear ratio." },
    { "drive.gear5", "Drivetrain", "", SPEC_ARRAY_OFFSET(gearRatios, 4), 0.82f, 0.4f, 6.0f,
      0.01f, false, false, 1, "Fifth gear ratio." },
    { "drive.reverse", "Drivetrain", "", SPEC_OFFSET(reverseGearRatio), REVERSE_GEAR_RATIO,
      0.4f, 6.0f, 0.01f, false, false, 1, "Reverse gear ratio." },
    { "drive.final", "Drivetrain", "", SPEC_OFFSET(finalDriveRatio), FINAL_DRIVE_RATIO, 1.0f,
      8.0f, 0.01f, false, false, 1, "Final drive ratio." },
    { "drive.efficiency", "Drivetrain", "", SPEC_OFFSET(drivetrainEfficiency),
      DRIVETRAIN_EFFICIENCY, 0.5f, 1.0f, 0.01f, false, false, 2, "Drivetrain efficiency." },
    { "drive.diff_mode", "Drivetrain", "", SPEC_OFFSET(differentialMode),
      (float)DIFFERENTIAL_MODE_DEFAULT, 0.0f, 2.0f, 1.0f, false, false, 1,
      "0=locked 1=open 2=LSD." },
    { "drive.diff_bias_ratio", "Drivetrain", "", SPEC_OFFSET(differentialBiasRatio), 2.0f, 1.0f,
      5.0f, 0.1f, false, false, 2, "LSD bias ratio." },
    { "drive.diff_preload", "Drivetrain", "N*m", SPEC_OFFSET(differentialPreloadNm), 60.0f,
      0.0f, 400.0f, 5.0f, false, false, 2, "LSD preload." },
    { "drive.layout", "Drivetrain", "", SPEC_OFFSET(drivetrainLayout),
      DRIVETRAIN_LAYOUT_DEFAULT, 0.0f, 2.0f, 1.0f, false, false, 0, "0=RWD 1=FWD 2=AWD." },
    { "drive.front_torque_split", "Drivetrain", "", SPEC_OFFSET(frontTorqueSplit),
      DRIVETRAIN_FRONT_TORQUE_SPLIT, 0.0f, 1.0f, 0.01f, false, false, 1,
      "AWD front torque share." },
    { "engine.idle_rpm", "Drivetrain", "rpm", SPEC_OFFSET(engineIdleRpm), ENGINE_IDLE_RPM,
      500.0f, 2000.0f, 25.0f, false, false, 1, "Idle RPM." },
    { "engine.redline_rpm", "Drivetrain", "rpm", SPEC_OFFSET(engineRedlineRpm),
      ENGINE_REDLINE_RPM, 3000.0f, 10000.0f, 100.0f, false, false, 0, "Redline RPM." },
    { "engine.torque_p0", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 0),
      140.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 0." },
    { "engine.torque_p1", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 1),
      200.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 1." },
    { "engine.torque_p2", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 2),
      240.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 2." },
    { "engine.torque_p3", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 3),
      255.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 3." },
    { "engine.torque_p4", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 4),
      250.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 4." },
    { "engine.torque_p5", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 5),
      230.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 5." },
    { "engine.torque_p6", "Drivetrain", "N*m", SPEC_ARRAY_OFFSET(engineTorqueCurveNm, 6),
      195.0f, 0.0f, 600.0f, 5.0f, false, false, 1, "Torque curve point 6." },
    { "engine.braking_torque", "Drivetrain", "N*m", SPEC_OFFSET(engineBrakingTorqueNm),
      ENGINE_BRAKING_TORQUE_NM, 0.0f, 200.0f, 1.0f, false, false, 2, "Engine braking torque." },
    { "engine.cylinders", "Drivetrain", "", SPEC_OFFSET(engineCylinders), ENGINE_CYLINDERS,
      2.0f, 12.0f, 1.0f, false, false, 1, "Cylinder count." },
    { "engine.displacement", "Drivetrain", "L", SPEC_OFFSET(engineDisplacementL),
      ENGINE_DISPLACEMENT_L, 0.5f, 8.0f, 0.1f, false, false, 1, "Displacement litres." },
    { "brake.max_torque", "Brakes", "N*m", SPEC_OFFSET(maxBrakeTorqueNm), MAX_BRAKE_TORQUE_NM,
      0.0f, 8000.0f, 50.0f, false, false, 0, "Total service brake torque." },
    { "brake.bias_front", "Brakes", "", SPEC_OFFSET(brakeBiasFront), BRAKE_BIAS_FRONT, 0.0f,
      1.0f, 0.01f, false, false, 1, "Front brake bias." },
    { "brake.handbrake_torque", "Brakes", "N*m", SPEC_OFFSET(handbrakeTorqueNm),
      HANDBRAKE_TORQUE_NM, 0.0f, 6000.0f, 50.0f, false, false, 1, "Handbrake torque." },
    { "brake.disc_radius_front", "Brakes", "m", SPEC_OFFSET(brakeDiscRadiusFrontM),
      BRAKE_DISC_RADIUS_FRONT_M, 0.08f, 0.22f, 0.005f, false, false, 2, "Front disc radius." },
    { "brake.disc_radius_rear", "Brakes", "m", SPEC_OFFSET(brakeDiscRadiusRearM),
      BRAKE_DISC_RADIUS_REAR_M, 0.08f, 0.22f, 0.005f, false, false, 2, "Rear disc radius." },
    { "brake.pad_friction", "Brakes", "", SPEC_OFFSET(brakePadFriction), BRAKE_PAD_FRICTION,
      0.15f, 0.6f, 0.01f, false, false, 2, "Pad friction coefficient." },
    { "aero.lift_front", "Aero", "", SPEC_OFFSET(aeroLiftCoefFront), AERO_LIFT_COEF_FRONT,
      -2.0f, 1.0f, 0.01f, false, false, 1, "Front lift coefficient." },
    { "aero.lift_rear", "Aero", "", SPEC_OFFSET(aeroLiftCoefRear), AERO_LIFT_COEF_REAR, -3.0f,
      1.0f, 0.01f, false, false, 1, "Rear lift coefficient." },
    { "aero.ref_area_front", "Aero", "m^2", SPEC_OFFSET(aeroRefAreaFrontM2),
      AERO_REF_AREA_FRONT_M2, 0.05f, 2.0f, 0.01f, false, false, 2,
      "Front aero reference area." },
    { "aero.ref_area_rear", "Aero", "m^2", SPEC_OFFSET(aeroRefAreaRearM2),
      AERO_REF_AREA_REAR_M2, 0.05f, 2.0f, 0.01f, false, false, 2, "Rear aero reference area." },
    { "aero.cop_x", "Aero", "m", SPEC_OFFSET(aeroCentreOfPressureXM), AERO_COP_X_M, -2.0f, 2.0f,
      0.01f, false, false, 2, "Centre of pressure X." },
    { "collision.half_width", "Collision", "m", SPEC_OFFSET(bodyHalfWidthM),
      VEHICLE_BODY_HALF_WIDTH_M, 0.4f, 1.5f, 0.01f, true, true, 1,
      "Collision capsule half-width = width/2 (derived)." },
    { "collision.restitution", "Collision", "", SPEC_OFFSET(collisionRestitution),
      COLLISION_RESTITUTION, 0.0f, 0.9f, 0.01f, false, false, 2, "Barrier restitution." },
    { "collision.friction", "Collision", "", SPEC_OFFSET(collisionFriction), COLLISION_FRICTION,
      0.0f, 1.5f, 0.01f, false, false, 2, "Barrier friction." },
};

#define PARAM_COUNT ((int)(sizeof(g_params) / sizeof(g_params[0])))

/* ------------------------------------------------------------------------------- access -- */

int dev_params_count(void)
{
    return PARAM_COUNT;
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
            if (strcmp(g_params[i].group, g_params[j].group) == 0) {
                seen = true;
                break;
            }
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
            if (strcmp(g_params[i].group, g_params[j].group) == 0) {
                seen = true;
                break;
            }
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
    vehicle_spec_refresh_derived(spec);
}

static float normalize_discrete_value(const DevParameter *param, float value)
{
    if (param == NULL) return value;
    if (strcmp(param->name, "body.side_window_count") == 0 ||
        strcmp(param->name, "body.cabin_rows") == 0 ||
        strcmp(param->name, "body.roof_type") == 0) {
        return roundf(value);
    }
    if (strcmp(param->name, "body.door_count") == 0) {
        const float rounded = roundf(value);
        return rounded == 3.0f ? 4.0f : rounded;
    }
    if (strcmp(param->name, "body.quarter_window") == 0 && value > 0.0f && value < 0.2f)
        return 0.2f;
    if (strcmp(param->name, "body.sunroof_length") == 0 && value > 0.0f && value < 0.4f)
        return 0.4f;
    return value;
}

bool dev_param_set(VehicleSpec *spec, const DevParameter *param, float value)
{
    if (spec == NULL || param == NULL) return false;
    if (param->derived) return false;
    if (!isfinite(value)) return false;
    if (value < param->minimum) value = param->minimum;
    if (value > param->maximum) value = param->maximum;
    *spec_field(spec, param) = normalize_discrete_value(param, value);
    dev_params_refresh_derived(spec);
    return true;
}

bool dev_param_is_default(const VehicleSpec *spec, const DevParameter *param)
{
    if (spec == NULL || param == NULL) return true;
    const float value = dev_param_get(spec, param);
    const float span = fmaxf(fabsf(param->defaultValue), 1.0f);
    return fabsf(value - param->defaultValue) <= span * 1e-5f;
}

void dev_param_reset(VehicleSpec *spec, const DevParameter *param)
{
    if (spec == NULL || param == NULL || param->derived) return;
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
        if (g_params[i].derived) continue;
        if (!dev_param_is_default(spec, &g_params[i])) count++;
    }
    return count;
}

/* -------------------------------------------------------------------- migration aliases -- */

static void scale_particle_masses(VehicleSpec *spec, float targetMassKg)
{
    const float current = spec->massEngineKg + spec->massGearboxKg + spec->massFuelKg +
                          spec->massDriverKg + spec->massChassisKg;
    if (!(current > 0.0f) || !(targetMassKg > 0.0f)) return;
    const float scale = targetMassKg / current;
    spec->massEngineKg *= scale;
    spec->massGearboxKg *= scale;
    spec->massFuelKg *= scale;
    spec->massDriverKg *= scale;
    spec->massChassisKg *= scale;
}

static void shift_particle_x_to_cg(VehicleSpec *spec, float targetCgToFrontM, float wheelbaseM)
{
    const float targetXcg = 0.5f * wheelbaseM - targetCgToFrontM;
    const float masses[5] = { spec->massEngineKg, spec->massGearboxKg, spec->massFuelKg,
                              spec->massDriverKg, spec->massChassisKg };
    float *xs[5] = { &spec->massEngineXM, &spec->massGearboxXM, &spec->massFuelXM,
                     &spec->massDriverXM, &spec->massChassisXM };
    float mass = 0.0f, moment = 0.0f;
    for (int i = 0; i < 5; i++) {
        mass += masses[i];
        moment += masses[i] * (*xs[i]);
    }
    if (!(mass > 0.0f)) return;
    const float shift = targetXcg - (moment / mass);
    for (int i = 0; i < 5; i++) *xs[i] += shift;
}

static void shift_particle_z_to_cg(VehicleSpec *spec, float targetHeightM)
{
    const float masses[5] = { spec->massEngineKg, spec->massGearboxKg, spec->massFuelKg,
                              spec->massDriverKg, spec->massChassisKg };
    float *zs[5] = { &spec->massEngineZM, &spec->massGearboxZM, &spec->massFuelZM,
                     &spec->massDriverZM, &spec->massChassisZM };
    float mass = 0.0f, moment = 0.0f;
    for (int i = 0; i < 5; i++) {
        mass += masses[i];
        moment += masses[i] * (*zs[i]);
    }
    if (!(mass > 0.0f)) return;
    const float shift = targetHeightM - (moment / mass);
    for (int i = 0; i < 5; i++) *zs[i] += shift;
}

static void set_loaded_radius_both_axles(VehicleSpec *spec, float targetRadiusM)
{
    if (!(targetRadiusM > 0.0f)) return;
    const float unloaded = targetRadiusM / TIRE_LOAD_RADIUS_FACTOR;
    /* Keep section/aspect; solve rim diameter (inches). */
    const float sidewallF =
        (spec->tireSectionWidthFrontMm * 0.001f) * (spec->tireAspectFrontPct * 0.01f);
    const float sidewallR =
        (spec->tireSectionWidthRearMm * 0.001f) * (spec->tireAspectRearPct * 0.01f);
    const float rimF = fmaxf(12.0f, (unloaded - sidewallF) * 2.0f / 0.0254f);
    const float rimR = fmaxf(12.0f, (unloaded - sidewallR) * 2.0f / 0.0254f);
    spec->tireRimDiameterFrontIn = rimF;
    spec->tireRimDiameterRearIn = rimR;
}

/* Apply one assignment, migrating legacy derived keys onto primaries. Returns:
 * 1 applied, 0 unknown, -1 rejected. */
static int apply_one_assignment(VehicleSpec *spec, const char *key, float value,
                                float *yawOverrideOut, bool *haveYawOverride)
{
    if (spec == NULL || key == NULL || !isfinite(value)) return -1;

    if (strcmp(key, "body.mass") == 0) {
        scale_particle_masses(spec, value);
        return 1;
    }
    if (strcmp(key, "body.cg_to_front") == 0) {
        /* Pair with existing cg_to_rear to set wheelbase, then shift particles. */
        const float lr = spec->cgToRearM > 0.0f ? spec->cgToRearM : (spec->wheelbaseM - value);
        if (value > 0.0f && lr > 0.0f) {
            spec->wheelbaseM = value + lr;
            shift_particle_x_to_cg(spec, value, spec->wheelbaseM);
        }
        return 1;
    }
    if (strcmp(key, "body.cg_to_rear") == 0) {
        const float lf =
            spec->cgToFrontM > 0.0f ? spec->cgToFrontM : (spec->wheelbaseM - value);
        if (lf > 0.0f && value > 0.0f) {
            spec->wheelbaseM = lf + value;
            shift_particle_x_to_cg(spec, lf, spec->wheelbaseM);
        }
        return 1;
    }
    if (strcmp(key, "body.cg_height") == 0) {
        shift_particle_z_to_cg(spec, value);
        return 1;
    }
    if (strcmp(key, "body.yaw_inertia") == 0) {
        if (yawOverrideOut != NULL && haveYawOverride != NULL) {
            *yawOverrideOut = value;
            *haveYawOverride = true;
        }
        return 1;
    }
    if (strcmp(key, "wheel.radius") == 0) {
        set_loaded_radius_both_axles(spec, value);
        return 1;
    }
    if (strcmp(key, "body.frontal_area") == 0) {
        if (spec->widthOverallM > 0.0f && VEH_FRONTAL_AREA_FILL > 0.0f) {
            spec->heightOverallM = value / (spec->widthOverallM * VEH_FRONTAL_AREA_FILL);
        }
        return 1;
    }
    if (strcmp(key, "collision.half_width") == 0) {
        spec->widthOverallM = 2.0f * value;
        return 1;
    }
    if (strcmp(key, "body.length_overall") == 0 || strcmp(key, "wheel.radius_front") == 0 ||
        strcmp(key, "wheel.radius_rear") == 0 || strcmp(key, "tire.load_ref_per_wheel") == 0) {
        /* Pure readouts with no clean primary inverse — accept as no-op applied. */
        return 1;
    }

    const DevParameter *param = dev_param_find(key);
    if (param == NULL) return 0;
    if (param->derived) {
        /* Unknown derived without an alias — treat as applied no-op so old profiles load. */
        return 1;
    }
    if (value < param->minimum) value = param->minimum;
    if (value > param->maximum) value = param->maximum;
    *spec_field(spec, param) = normalize_discrete_value(param, value);
    return 1;
}

int dev_params_apply_assignments(VehicleSpec *spec, const DevParamAssignment *items, int count,
                                 int *unknownOut, int *rejectedOut)
{
    int applied = 0;
    int unknown = 0;
    int rejected = 0;
    float yawOverride = 0.0f;
    bool haveYawOverride = false;

    if (spec == NULL || items == NULL || count <= 0) {
        if (unknownOut) *unknownOut = 0;
        if (rejectedOut) *rejectedOut = 0;
        return 0;
    }

    /* Two-pass for cg_to_front/rear so both are visible when setting wheelbase. */
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < count; i++) {
            const char *key = items[i].key;
            if (key == NULL) continue;
            const bool isCg =
                (strcmp(key, "body.cg_to_front") == 0 || strcmp(key, "body.cg_to_rear") == 0);
            if (pass == 0 && isCg) {
                /* Seed cg distances so the paired alias can read the other side. */
                if (strcmp(key, "body.cg_to_front") == 0)
                    spec->cgToFrontM = items[i].value;
                else
                    spec->cgToRearM = items[i].value;
                continue;
            }
            if (pass == 0 && !isCg) {
                const int status = apply_one_assignment(spec, key, items[i].value, &yawOverride,
                                                        &haveYawOverride);
                if (status > 0)
                    applied++;
                else if (status == 0)
                    unknown++;
                else
                    rejected++;
            }
            if (pass == 1 && isCg) {
                const int status = apply_one_assignment(spec, key, items[i].value, &yawOverride,
                                                        &haveYawOverride);
                if (status > 0)
                    applied++;
                else if (status == 0)
                    unknown++;
                else
                    rejected++;
            }
        }
    }

    vehicle_spec_refresh_derived(spec);
    if (haveYawOverride) spec->yawInertiaKgM2 = yawOverride;

    if (unknownOut) *unknownOut = unknown;
    if (rejectedOut) *rejectedOut = rejected;
    return applied;
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
        if (param->derived) continue; /* profiles store primaries only */
        if (group == NULL || strcmp(group, param->group) != 0) {
            group = param->group;
            fprintf(file, "\n# --- %s ---\n", group);
        }
        fprintf(file, "%-28s = %-12.6f # %s%s(default %.6f)\n", param->name,
                (double)dev_param_get(spec, param), param->unit[0] != '\0' ? param->unit : "",
                param->unit[0] != '\0' ? " " : "", (double)param->defaultValue);
    }

    const bool ok = (fflush(file) == 0) && !ferror(file);
    return (fclose(file) == 0) && ok;
}

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

    char valueBuffer[64];
    size_t valueLength = 0;
    while (i < length && valueLength + 1 < sizeof(valueBuffer) && line[i] != '#' &&
           line[i] != ';' && line[i] != '\r' && line[i] != '\n') {
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

bool dev_params_apply_text(VehicleSpec *spec, const char *text, size_t length, int *appliedOut,
                           int *unknownOut, int *rejectedOut)
{
    int applied = 0;
    int unknown = 0;
    int rejected = 0;

    if (appliedOut != NULL) *appliedOut = 0;
    if (unknownOut != NULL) *unknownOut = 0;
    if (rejectedOut != NULL) *rejectedOut = 0;
    if (spec == NULL || text == NULL) return false;

    VehicleSpec candidate = *spec;

    /* Collect assignments first so cg_front/rear migrate together. */
    DevParamAssignment *items = NULL;
    int itemCount = 0;
    int itemCap = 0;

    size_t offset = 0;
    while (offset < length) {
        size_t end = offset;
        while (end < length && text[end] != '\n') end++;

        char name[64];
        float value = 0.0f;
        const int status = parse_line(text + offset, end - offset, name, sizeof(name), &value);
        if (status == 1) {
            if (itemCount >= itemCap) {
                const int newCap = itemCap == 0 ? 64 : itemCap * 2;
                DevParamAssignment *grown =
                    (DevParamAssignment *)realloc(items, (size_t)newCap * sizeof(*items));
                if (grown == NULL) {
                    free(items);
                    return false;
                }
                items = grown;
                itemCap = newCap;
            }
            /* Store durable copies of names in a side buffer — profile lines are temporary.
             * Use a simple name arena on the heap. */
            char *nameCopy = (char *)malloc(strlen(name) + 1u);
            if (nameCopy == NULL) {
                free(items);
                return false;
            }
            memcpy(nameCopy, name, strlen(name) + 1u);
            items[itemCount].key = nameCopy;
            items[itemCount].value = value;
            itemCount++;
        } else if (status < 0) {
            rejected++;
        }

        offset = (end < length) ? end + 1 : length;
    }

    int assignUnknown = 0;
    int assignRejected = 0;
    applied = dev_params_apply_assignments(&candidate, items, itemCount, &assignUnknown,
                                           &assignRejected);
    unknown += assignUnknown;
    rejected += assignRejected;

    for (int i = 0; i < itemCount; i++) free((void *)items[i].key);
    free(items);

    if (!vehicle_spec_is_valid(&candidate)) return false;

    *spec = candidate;
    if (appliedOut != NULL) *appliedOut = applied;
    if (unknownOut != NULL) *unknownOut = unknown;
    if (rejectedOut != NULL) *rejectedOut = rejected;
    return true;
}

bool dev_params_load(VehicleSpec *spec, const char *path, int *appliedOut, int *unknownOut,
                     int *rejectedOut)
{
    if (spec == NULL || path == NULL) return false;

    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;

    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return false;
    }
    const long size = ftell(file);
    if (size < 0 || size > (1L << 20)) {
        fclose(file);
        return false;
    }
    if (fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return false;
    }

    char *buffer = (char *)malloc((size_t)size + 1u);
    if (buffer == NULL) {
        fclose(file);
        return false;
    }

    const size_t read = fread(buffer, 1, (size_t)size, file);
    fclose(file);
    buffer[read] = '\0';

    const bool ok =
        dev_params_apply_text(spec, buffer, read, appliedOut, unknownOut, rejectedOut);
    free(buffer);
    return ok;
}

/* ------------------------------------------------------------------------------ reports -- */

void dev_params_write_markdown(FILE *out)
{
    if (out == NULL) return;

    fprintf(out, "<!-- Generated by `build/tests/circuit_tests.exe --dump-params`. Do not edit "
                 "by hand. -->\n");
    fprintf(out, "# Tunable parameters\n\n");
    fprintf(
        out,
        "Every row is one entry in the `src/dev/dev_params.c` registry, which is also what\n");
    fprintf(out, "generates the Physics Lab sliders and the tuning-profile format.\n");

    const int groups = dev_params_group_count();
    for (int g = 0; g < groups; g++) {
        const char *group = dev_params_group_name(g);
        fprintf(out, "\n## %s\n\n", group);
        fprintf(out, "| Parameter | Default | Unit | Range | Step | Tier | Derived | Reset | "
                     "Meaning |\n");
        fprintf(out, "|---|---:|---|---|---:|---|:-:|:-:|---|\n");
        for (int i = 0; i < PARAM_COUNT; i++) {
            const DevParameter *param = &g_params[i];
            if (strcmp(param->group, group) != 0) continue;
            const char *tierName = param->tier == DEV_TIER_ESSENTIAL  ? "essential"
                                   : param->tier == DEV_TIER_ADVANCED ? "advanced"
                                                                      : "expert";
            fprintf(out, "| `%s` | %g | %s | %g .. %g | %g | %s | %s | %s | %s |\n",
                    param->name, (double)param->defaultValue,
                    param->unit[0] != '\0' ? param->unit : "—", (double)param->minimum,
                    (double)param->maximum, (double)param->step, tierName,
                    param->derived ? "yes" : "—", param->requiresRestart ? "yes" : "—",
                    param->description);
        }
    }

    fprintf(out,
            "\n`Reset` marks a parameter that only takes full effect after a simulation\n");
    fprintf(out, "reset, because it moves the wheel contact points. `Derived` rows are\n");
    fprintf(out, "read-only; write the primaries that produce them.\n");
}

void dev_params_write_metadata(FILE *out, const VehicleSpec *spec)
{
    if (out == NULL || spec == NULL) return;
    fprintf(out, "# parameter,value,default,unit\n");
    for (int i = 0; i < PARAM_COUNT; i++) {
        const DevParameter *param = &g_params[i];
        fprintf(out, "# %s,%.6f,%.6f,%s\n", param->name, (double)dev_param_get(spec, param),
                (double)param->defaultValue, param->unit[0] != '\0' ? param->unit : "-");
    }
}

int dev_params_write_overrides(FILE *out, const VehicleSpec *spec)
{
    if (out == NULL || spec == NULL) return 0;
    int written = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        const DevParameter *param = &g_params[i];
        if (param->derived) continue;
        if (dev_param_is_default(spec, param)) continue;
        fprintf(out, "%s%s=%.6f", (written > 0) ? " " : "", param->name,
                (double)dev_param_get(spec, param));
        written++;
    }
    return written;
}
