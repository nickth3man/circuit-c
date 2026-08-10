/*
 * car_roster.c — data-driven vehicle roster loaded from data/vehicles/ (Issue #30).
 */
#include "game/car_roster.h"

#include <stdio.h>
#include <string.h>

#include "content/vehicle_manifest.h"
#include "physics/vehicle.h"

static VehicleCatalog g_rosterCatalog;
static bool g_rosterLoaded = false;

static bool ensure_roster_loaded(void)
{
    if (g_rosterLoaded) return (g_rosterCatalog.count > 0);
    char error[256];
    if (!vehicle_manifest_load_dir("data/vehicles", &g_rosterCatalog, error, sizeof(error))) {
        return false;
    }
    g_rosterLoaded = true;
    return (g_rosterCatalog.count > 0);
}

void car_roster_reload(void)
{
    if (g_rosterLoaded) {
        vehicle_catalog_free(&g_rosterCatalog);
        g_rosterLoaded = false;
    }
}

int car_roster_count(void)
{
    if (!ensure_roster_loaded()) return 0;
    return g_rosterCatalog.count;
}

bool car_roster_spec(int index, VehicleSpec *out)
{
    if (out == NULL || !ensure_roster_loaded()) return false;
    if (index < 0 || index >= g_rosterCatalog.count) return false;
    *out = g_rosterCatalog.items[index].definition.spec;
    return true;
}

void car_roster_id(int index, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    if (!ensure_roster_loaded() || index < 0 || index >= g_rosterCatalog.count) {
        buf[0] = '\0';
        return;
    }
    snprintf(buf, cap, "%s", g_rosterCatalog.items[index].definition.id);
}

const char *car_roster_display_name(int index)
{
    if (!ensure_roster_loaded() || index < 0 || index >= g_rosterCatalog.count) return "?";
    return g_rosterCatalog.items[index].displayName;
}

const char *car_roster_describe(int index)
{
    if (!ensure_roster_loaded() || index < 0 || index >= g_rosterCatalog.count) return "";
    return g_rosterCatalog.items[index].description;
}

const char *car_roster_layout_name(int index)
{
    VehicleSpec spec;
    if (!car_roster_spec(index, &spec)) return "?";
    switch ((DrivetrainLayout)(int)spec.drivetrainLayout) {
        case DRIVE_LAYOUT_FWD: return "FWD";
        case DRIVE_LAYOUT_AWD: return "AWD";
        case DRIVE_LAYOUT_RWD:
        default: return "RWD";
    }
}

uint32_t car_roster_spec_hash(int index)
{
    VehicleSpec spec;
    if (!car_roster_spec(index, &spec)) return 0;

    const unsigned char *bytes = (const unsigned char *)&spec;
    uint32_t hash = 0x811c9dc5u;
    for (size_t i = 0; i < sizeof(spec); i++) {
        hash ^= bytes[i];
        hash *= 0x01000193u;
    }
    return hash;
}

int car_roster_find(const char *id)
{
    if (id == NULL || !ensure_roster_loaded()) return -1;
    for (int i = 0; i < g_rosterCatalog.count; i++) {
        if (strcmp(g_rosterCatalog.items[i].definition.id, id) == 0) return i;
    }
    return -1;
}
