#!/usr/bin/env python3
"""One-shot generator for Phase 2 g_params[] initializers. Not used at runtime."""
ESS, ADV, EXP = 0, 1, 2
rows = []


def P(name, group, unit, field, default, lo, hi, step, restart, derived, tier, desc):
    rows.append((name, group, unit, field, default, lo, hi, step, restart, derived, tier, desc))


P("body.wheelbase", "Body", "m", "wheelbaseM", "VEH_WHEELBASE_M", 1.80, 7.00, 0.01, True, False, ESS,
  "Wheelbase; primary. CG distances derive from this plus mass particles.")
P("body.track_front", "Body", "m", "trackWidthFrontM", "VEH_TRACK_FRONT_M", 1.00, 2.60, 0.01, True, False, ESS,
  "Front track width.")
P("body.track_rear", "Body", "m", "trackWidthRearM", "VEH_TRACK_REAR_M", 1.00, 2.60, 0.01, True, False, ESS,
  "Rear track width.")
P("body.front_overhang", "Body", "m", "frontOverhangM", "VEH_FRONT_OVERHANG_M", 0.20, 2.50, 0.01, False, False, ADV,
  "Body ahead of the front axle.")
P("body.rear_overhang", "Body", "m", "rearOverhangM", "VEH_REAR_OVERHANG_M", 0.20, 2.50, 0.01, False, False, ADV,
  "Body behind the rear axle.")
P("body.width_overall", "Body", "m", "widthOverallM", "VEH_WIDTH_OVERALL_M", 1.20, 2.60, 0.01, True, False, ESS,
  "Overall body width; derives collision half-width.")
P("body.height_overall", "Body", "m", "heightOverallM", "VEH_HEIGHT_OVERALL_M", 1.00, 3.20, 0.01, False, False, ESS,
  "Overall body height; derives frontal area with width.")
P("body.ride_height_front", "Body", "m", "rideHeightFrontM", "VEH_RIDE_HEIGHT_FRONT_M", 0.04, 0.50, 0.005, False, False, ADV,
  "Front ride height / arch clearance.")
P("body.ride_height_rear", "Body", "m", "rideHeightRearM", "VEH_RIDE_HEIGHT_REAR_M", 0.04, 0.50, 0.005, False, False, ADV,
  "Rear ride height / arch clearance.")
P("body.cowl_x", "Body", "m", "cowlXM", "VEH_COWL_X_M", -2.0, 2.0, 0.01, False, False, ADV,
  "Cowl X in layout frame (axle midpoint origin).")
P("body.backlight_x", "Body", "m", "backlightXM", "VEH_BACKLIGHT_X_M", -2.0, 2.0, 0.01, False, False, ADV,
  "Backlight X in layout frame.")
P("body.drag_coefficient", "Body", "", "dragCoefficient", "DRAG_COEFFICIENT", 0.10, 1.20, 0.01, False, False, ADV,
  "Cd in 0.5*rho*Cd*A*v^2.")
P("body.rolling_resistance", "Body", "", "rollingResistanceCoefficient", "ROLLING_RESISTANCE_COEF", 0.000, 0.060, 0.001, False, False, ADV,
  "Rolling resistance coefficient.")
P("body.load_filter_rate", "Body", "Hz", "loadFilterRateHz", "LOAD_FILTER_RATE_HZ", 1.0, 60.0, 0.5, False, False, EXP,
  "Load-transfer accel filter corner frequency.")
P("body.roll_stiffness_front", "Body", "", "rollStiffnessFrontFraction", "ROLL_STIFFNESS_FRONT_FRACTION", 0.00, 1.00, 0.01, False, False, ADV,
  "Front axle share of roll moment.")
P("body.mass", "Body", "kg", "massKg", "VEH_MASS_KG", 400.0, 15000.0, 10.0, False, True, ESS,
  "Total mass from particles (read-only).")
P("body.yaw_inertia", "Body", "kg*m^2", "yawInertiaKgM2", "VEH_YAW_INERTIA_KGM2", 200.0, 40000.0, 25.0, False, True, ADV,
  "Yaw inertia from particles (read-only).")
P("body.cg_to_front", "Body", "m", "cgToFrontM", "VEH_CG_TO_FRONT_M", 0.40, 4.00, 0.01, True, True, ESS,
  "CG to front axle (derived).")
P("body.cg_to_rear", "Body", "m", "cgToRearM", "VEH_CG_TO_REAR_M", 0.40, 4.00, 0.01, True, True, ESS,
  "CG to rear axle (derived).")
P("body.cg_height", "Body", "m", "cgHeightM", "VEH_CG_HEIGHT_M", 0.15, 1.50, 0.01, False, True, ADV,
  "CG height (derived).")
P("body.frontal_area", "Body", "m^2", "frontalAreaM2", "FRONTAL_AREA_M2", 1.00, 8.00, 0.05, False, True, ADV,
  "Frontal area from width*height*fill (derived).")
P("body.length_overall", "Body", "m", "lengthOverallM", "VEH_LENGTH_OVERALL_M", 2.5, 12.0, 0.01, False, True, ADV,
  "Overall length = wheelbase + overhangs.")

for part, field_m, field_x, field_z, dm, dx, dz in [
    ("engine", "massEngineKg", "massEngineXM", "massEngineZM", "MASS_ENGINE_KG", "MASS_ENGINE_X_M", "MASS_ENGINE_Z_M"),
    ("gearbox", "massGearboxKg", "massGearboxXM", "massGearboxZM", "MASS_GEARBOX_KG", "MASS_GEARBOX_X_M", "MASS_GEARBOX_Z_M"),
    ("fuel", "massFuelKg", "massFuelXM", "massFuelZM", "MASS_FUEL_KG", "MASS_FUEL_X_M", "MASS_FUEL_Z_M"),
    ("driver", "massDriverKg", "massDriverXM", "massDriverZM", "MASS_DRIVER_KG", "MASS_DRIVER_X_M", "MASS_DRIVER_Z_M"),
    ("chassis", "massChassisKg", "massChassisXM", "massChassisZM", "MASS_CHASSIS_KG", "MASS_CHASSIS_X_M", "MASS_CHASSIS_Z_M"),
]:
    tier_m = ESS if part in ("engine", "chassis") else ADV
    P(f"mass.{part}", "Mass", "kg", field_m, dm, 20.0, 8000.0, 5.0, False, False, tier_m, f"{part.capitalize()} mass particle.")
    P(f"mass.{part}_x", "Mass", "m", field_x, dx, -4.0, 4.0, 0.01, False, False, ESS if part == "engine" else ADV,
      f"{part.capitalize()} X in layout frame.")
    P(f"mass.{part}_z", "Mass", "m", field_z, dz, 0.05, 2.00, 0.01, False, False, EXP,
      f"{part.capitalize()} Z (height) in layout frame.")

P("wheel.inertia", "Wheels", "kg*m^2", "wheelInertiaKgM2", "WHEEL_INERTIA_KGM2", 0.30, 8.00, 0.05, False, False, ADV,
  "Rotational inertia of one wheel.")
P("wheel.radius", "Wheels", "m", "wheelRadiusM", "WHEEL_RADIUS_M", 0.15, 0.60, 0.005, False, True, ADV,
  "Legacy rear rolling radius (derived).")
P("wheel.radius_front", "Wheels", "m", "wheelRadiusFrontM", "WHEEL_RADIUS_M", 0.15, 0.60, 0.005, False, True, ESS,
  "Front loaded rolling radius (derived).")
P("wheel.radius_rear", "Wheels", "m", "wheelRadiusRearM", "WHEEL_RADIUS_M", 0.15, 0.60, 0.005, False, True, ESS,
  "Rear loaded rolling radius (derived).")
P("wheel.offset_et_front", "Wheels", "mm", "wheelOffsetEtFrontMm", "WHEEL_OFFSET_ET_FRONT_MM", -20.0, 60.0, 1.0, False, False, EXP,
  "Front wheel ET offset.")
P("wheel.offset_et_rear", "Wheels", "mm", "wheelOffsetEtRearMm", "WHEEL_OFFSET_ET_REAR_MM", -20.0, 60.0, 1.0, False, False, EXP,
  "Rear wheel ET offset.")

for axle, suf in [("front", "Front"), ("rear", "Rear")]:
    P(f"tire.section_width_{axle}", "Tires", "mm", f"tireSectionWidth{suf}Mm", "TIRE_SECTION_WIDTH_MM",
      145.0, 355.0, 5.0, False, False, ESS, f"{suf} tire section width.")
    P(f"tire.aspect_{axle}", "Tires", "%", f"tireAspect{suf}Pct", "TIRE_ASPECT_RATIO_PCT",
      25.0, 80.0, 1.0, False, False, ADV, f"{suf} tire aspect ratio.")
    P(f"tire.rim_diameter_{axle}", "Tires", "in", f"tireRimDiameter{suf}In", "TIRE_RIM_DIAMETER_IN",
      12.0, 22.0, 0.5, False, False, ADV, f"{suf} rim diameter.")
    P(f"tire.rim_width_{axle}", "Tires", "in", f"tireRimWidth{suf}In", "TIRE_RIM_WIDTH_IN",
      4.0, 14.0, 0.5, False, False, EXP, f"{suf} rim width.")
    P(f"tire.pressure_{axle}", "Tires", "kPa", f"tirePressure{suf}Kpa", "TIRE_PRESSURE_KPA",
      120.0, 400.0, 5.0, False, False, EXP, f"{suf} cold pressure.")

for name, field, default, lo, hi, step, unit, tier, desc in [
    ("tire.lat_front.b", "tireBLatFront", "TIRE_B_LAT_FRONT", 2.0, 25.0, 0.1, "", ADV, "Front lateral stiffness factor."),
    ("tire.lat_front.c", "tireCLatFront", "TIRE_C_LAT_FRONT", 1.00, 2.20, 0.01, "", ADV, "Front lateral shape factor."),
    ("tire.lat_front.mu", "tireMuLatFront", "TIRE_MU_LAT_FRONT", 0.30, 2.00, 0.01, "", ADV, "Front lateral peak friction."),
    ("tire.lat_rear.b", "tireBLatRear", "TIRE_B_LAT_REAR", 2.0, 25.0, 0.1, "", ADV, "Rear lateral stiffness factor."),
    ("tire.lat_rear.c", "tireCLatRear", "TIRE_C_LAT_REAR", 1.00, 2.20, 0.01, "", ADV, "Rear lateral shape factor."),
    ("tire.lat_rear.mu", "tireMuLatRear", "TIRE_MU_LAT_REAR", 0.30, 2.00, 0.01, "", ADV, "Rear lateral peak friction."),
    ("tire.long.b", "tireBLong", "TIRE_B_LONG", 2.0, 30.0, 0.1, "", ADV, "Longitudinal stiffness factor."),
    ("tire.long.c", "tireCLong", "TIRE_C_LONG", 1.00, 2.20, 0.01, "", ADV, "Longitudinal shape factor."),
    ("tire.long.mu_scale", "tireMuLongScale", "TIRE_MU_LONG_SCALE", 0.30, 2.00, 0.01, "", ADV, "Longitudinal friction scale."),
    ("tire.relaxation_length", "tireRelaxationLengthM", "TIRE_RELAXATION_LENGTH_M", 0.00, 1.00, 0.01, "m", EXP, "Lateral force relaxation length."),
    ("tire.load_sensitivity_k", "tireLoadSensitivityK", "TIRE_LOAD_SENSITIVITY_K", 0.00, 0.05, 0.001, "", EXP, "Load sensitivity exponent."),
]:
    P(name, "Tires", unit, field, default, lo, hi, step, False, False, tier, desc)
P("tire.load_ref_per_wheel", "Tires", "N", "tireLoadRefPerWheelN", "TIRE_LOAD_REF_PER_WHEEL_N",
  500.0, 15000.0, 10.0, False, True, EXP, "Reference load = m*g/4 (derived).")

P("steer.max_angle", "Steering", "rad", "maxRoadWheelAngleRad", "STEER_MAX_RAD", 0.20, 1.20, 0.01, False, False, ESS, "Max road-wheel angle.")
P("steer.rate", "Steering", "rad/s", "maxSteerRateRadS", "STEER_RATE_RAD_S", 0.50, 20.0, 0.10, False, False, ADV, "Steer follow rate.")
P("steer.return_rate", "Steering", "rad/s", "steerReturnRateRadS", "STEER_RETURN_RATE_RAD_S", 0.50, 25.0, 0.10, False, False, ADV, "Steer return rate.")
P("steer.ackermann_percent", "Steering", "", "ackermannPercent", "ACKERMANN_PERCENT", 0.00, 1.00, 0.01, False, False, ADV, "Ackermann blend.")

for name, field, default, lo, hi, step, unit, tier, desc in [
    ("susp.camber_front", "suspCamberFrontRad", "SUSP_CAMBER_FRONT_RAD", -0.12, 0.05, 0.001, "rad", EXP, "Front static camber."),
    ("susp.camber_rear", "suspCamberRearRad", "SUSP_CAMBER_REAR_RAD", -0.12, 0.05, 0.001, "rad", EXP, "Rear static camber."),
    ("susp.toe_front", "suspToeFrontRad", "SUSP_TOE_FRONT_RAD", -0.05, 0.05, 0.001, "rad", EXP, "Front static toe."),
    ("susp.toe_rear", "suspToeRearRad", "SUSP_TOE_REAR_RAD", -0.05, 0.05, 0.001, "rad", EXP, "Rear static toe."),
    ("susp.caster_front", "suspCasterFrontRad", "SUSP_CASTER_FRONT_RAD", 0.0, 0.25, 0.001, "rad", EXP, "Front caster."),
    ("susp.caster_rear", "suspCasterRearRad", "SUSP_CASTER_REAR_RAD", 0.0, 0.25, 0.001, "rad", EXP, "Rear caster."),
    ("susp.wheel_rate_front", "suspWheelRateFrontNpm", "SUSP_WHEEL_RATE_FRONT_NPM", 5000.0, 80000.0, 100.0, "N/m", EXP, "Front wheel rate."),
    ("susp.wheel_rate_rear", "suspWheelRateRearNpm", "SUSP_WHEEL_RATE_REAR_NPM", 5000.0, 80000.0, 100.0, "N/m", EXP, "Rear wheel rate."),
    ("susp.anti_roll_front", "suspAntiRollFrontNpm", "SUSP_ANTI_ROLL_FRONT_NPM", 0.0, 60000.0, 100.0, "N/m", EXP, "Front anti-roll stiffness."),
    ("susp.anti_roll_rear", "suspAntiRollRearNpm", "SUSP_ANTI_ROLL_REAR_NPM", 0.0, 60000.0, 100.0, "N/m", EXP, "Rear anti-roll stiffness."),
    ("susp.travel_front", "suspTravelFrontM", "SUSP_TRAVEL_FRONT_M", 0.03, 0.25, 0.005, "m", ADV, "Front suspension travel."),
    ("susp.travel_rear", "suspTravelRearM", "SUSP_TRAVEL_REAR_M", 0.03, 0.25, 0.005, "m", ADV, "Rear suspension travel."),
    ("susp.roll_centre_front", "suspRollCentreFrontM", "SUSP_ROLL_CENTRE_FRONT_M", 0.0, 0.40, 0.005, "m", EXP, "Front roll-centre height."),
    ("susp.roll_centre_rear", "suspRollCentreRearM", "SUSP_ROLL_CENTRE_REAR_M", 0.0, 0.40, 0.005, "m", EXP, "Rear roll-centre height."),
]:
    P(name, "Suspension", unit, field, default, lo, hi, step, False, False, tier, desc)

for i, v in enumerate([3.55, 2.05, 1.38, 1.00, 0.82]):
    P(f"drive.gear{i+1}", "Drivetrain", "", f"gearRatios[{i}]", f"{v:.2f}f", 0.40, 6.00, 0.01, False, False, ADV,
      f"{['First','Second','Third','Fourth','Fifth'][i]} gear ratio.")
P("drive.reverse", "Drivetrain", "", "reverseGearRatio", "REVERSE_GEAR_RATIO", 0.40, 6.00, 0.01, False, False, ADV, "Reverse gear ratio.")
P("drive.final", "Drivetrain", "", "finalDriveRatio", "FINAL_DRIVE_RATIO", 1.00, 8.00, 0.01, False, False, ADV, "Final drive ratio.")
P("drive.efficiency", "Drivetrain", "", "drivetrainEfficiency", "DRIVETRAIN_EFFICIENCY", 0.50, 1.00, 0.01, False, False, EXP, "Drivetrain efficiency.")
P("drive.diff_mode", "Drivetrain", "", "differentialMode", "0.0f", 0.0, 2.0, 1.0, False, False, ADV, "0=locked 1=open 2=LSD.")
P("drive.diff_bias_ratio", "Drivetrain", "", "differentialBiasRatio", "2.0f", 1.0, 5.0, 0.1, False, False, EXP, "LSD bias ratio.")
P("drive.diff_preload", "Drivetrain", "N*m", "differentialPreloadNm", "60.0f", 0.0, 400.0, 5.0, False, False, EXP, "LSD preload.")
P("drive.layout", "Drivetrain", "", "drivetrainLayout", "DRIVETRAIN_LAYOUT_DEFAULT", 0.0, 2.0, 1.0, False, False, ESS, "0=RWD 1=FWD 2=AWD.")
P("drive.front_torque_split", "Drivetrain", "", "frontTorqueSplit", "DRIVETRAIN_FRONT_TORQUE_SPLIT", 0.0, 1.0, 0.01, False, False, ADV, "AWD front torque share.")
P("engine.idle_rpm", "Drivetrain", "rpm", "engineIdleRpm", "ENGINE_IDLE_RPM", 500.0, 2000.0, 25.0, False, False, ADV, "Idle RPM.")
P("engine.redline_rpm", "Drivetrain", "rpm", "engineRedlineRpm", "ENGINE_REDLINE_RPM", 3000.0, 10000.0, 100.0, False, False, ESS, "Redline RPM.")
for i, v in enumerate([140, 200, 240, 255, 250, 230, 195]):
    P(f"engine.torque_p{i}", "Drivetrain", "N*m", f"engineTorqueCurveNm[{i}]", f"{v}.0f", 0.0, 600.0, 5.0, False, False, ADV,
      f"Torque curve point {i}.")
P("engine.braking_torque", "Drivetrain", "N*m", "engineBrakingTorqueNm", "ENGINE_BRAKING_TORQUE_NM", 0.0, 200.0, 1.0, False, False, EXP, "Engine braking torque.")
P("engine.cylinders", "Drivetrain", "", "engineCylinders", "ENGINE_CYLINDERS", 2.0, 12.0, 1.0, False, False, ADV, "Cylinder count.")
P("engine.displacement", "Drivetrain", "L", "engineDisplacementL", "ENGINE_DISPLACEMENT_L", 0.5, 8.0, 0.1, False, False, ADV, "Displacement litres.")

P("brake.max_torque", "Brakes", "N*m", "maxBrakeTorqueNm", "MAX_BRAKE_TORQUE_NM", 0.0, 8000.0, 50.0, False, False, ESS, "Total service brake torque.")
P("brake.bias_front", "Brakes", "", "brakeBiasFront", "BRAKE_BIAS_FRONT", 0.0, 1.0, 0.01, False, False, ADV, "Front brake bias.")
P("brake.handbrake_torque", "Brakes", "N*m", "handbrakeTorqueNm", "HANDBRAKE_TORQUE_NM", 0.0, 6000.0, 50.0, False, False, ADV, "Handbrake torque.")
P("brake.disc_radius_front", "Brakes", "m", "brakeDiscRadiusFrontM", "BRAKE_DISC_RADIUS_FRONT_M", 0.08, 0.22, 0.005, False, False, EXP, "Front disc radius.")
P("brake.disc_radius_rear", "Brakes", "m", "brakeDiscRadiusRearM", "BRAKE_DISC_RADIUS_REAR_M", 0.08, 0.22, 0.005, False, False, EXP, "Rear disc radius.")
P("brake.pad_friction", "Brakes", "", "brakePadFriction", "BRAKE_PAD_FRICTION", 0.15, 0.60, 0.01, False, False, EXP, "Pad friction coefficient.")

P("aero.lift_front", "Aero", "", "aeroLiftCoefFront", "AERO_LIFT_COEF_FRONT", -2.0, 1.0, 0.01, False, False, ADV, "Front lift coefficient.")
P("aero.lift_rear", "Aero", "", "aeroLiftCoefRear", "AERO_LIFT_COEF_REAR", -3.0, 1.0, 0.01, False, False, ADV, "Rear lift coefficient.")
P("aero.ref_area_front", "Aero", "m^2", "aeroRefAreaFrontM2", "AERO_REF_AREA_FRONT_M2", 0.05, 2.0, 0.01, False, False, EXP, "Front aero reference area.")
P("aero.ref_area_rear", "Aero", "m^2", "aeroRefAreaRearM2", "AERO_REF_AREA_REAR_M2", 0.05, 2.0, 0.01, False, False, EXP, "Rear aero reference area.")
P("aero.cop_x", "Aero", "m", "aeroCentreOfPressureXM", "AERO_COP_X_M", -2.0, 2.0, 0.01, False, False, EXP, "Centre of pressure X.")

P("collision.half_width", "Collision", "m", "bodyHalfWidthM", "VEHICLE_BODY_HALF_WIDTH_M", 0.40, 1.50, 0.01, True, True, ADV,
  "Collision capsule half-width = width/2 (derived).")
P("collision.restitution", "Collision", "", "collisionRestitution", "COLLISION_RESTITUTION", 0.00, 0.90, 0.01, False, False, EXP, "Barrier restitution.")
P("collision.friction", "Collision", "", "collisionFriction", "COLLISION_FRICTION", 0.00, 1.50, 0.01, False, False, EXP, "Barrier friction.")

print(f"static const DevParameter g_params[] = {{")
for name, group, unit, field, default, lo, hi, step, restart, derived, tier, desc in rows:
    if "[" in field:
        base = field[: field.index("[")]
        idx = field[field.index("[") + 1 : field.index("]")]
        off = f"SPEC_ARRAY_OFFSET({base}, {idx})"
    else:
        off = f"SPEC_OFFSET({field})"
    step_s = f"{step}f" if isinstance(step, float) else f"{step}.0f"
    if step == int(step):
        step_s = f"{int(step)}.0f" if step >= 1 else f"{step}f"
    print(f'    {{ "{name}", "{group}", "{unit}", {off},')
    print(f"      {default}, {lo}f, {hi}f, {step}f, "
          f"{'true' if restart else 'false'}, {'true' if derived else 'false'}, {tier},")
    print(f'      "{desc}" }},')
print("};")
print(f"\n/* PARAM_COUNT = {len(rows)} */", file=__import__("sys").stderr)
