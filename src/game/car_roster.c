/*
 * car_roster.c — data-driven vehicle roster loaded from data/vehicles/ (Issue #30).
 *
 * THE ROSTER IS THE GATE. Everything the game races comes through here, so this is where the
 * issue #31 promotion checklist and the issue #33 class rules are actually enforced. A manifest
 * that merely parses is not race content: it must declare itself player-selectable, pass every
 * promotion check, and — for each class tag naming a class the catalogue actually defines —
 * satisfy that class's numeric rules. Anything that fails is kept out of the roster and recorded
 * in the rejection log below, so a content author gets an explainable "why is my car missing?"
 * instead of a silently absent entry.
 *
 * WHY CLASSES ONLY CONSTRAIN WHEN THEY EXIST. A class tag is also a plain grouping label. A tag
 * with no matching data/vehicles/classes/<id>.vehicle-class.json is treated as unconstrained,
 * because the alternative — rejecting every car whose tag has no reviewed rule file — would
 * empty the roster the moment a packaged build shipped without the class directory. When a class
 * file does exist, its rules are binding: that is what turns a display string into a contract.
 */
#include "game/car_roster.h"

#include <stdio.h>
#include <string.h>

#include "content/roster_promotion.h"
#include "content/vehicle_class.h"
#include "content/vehicle_manifest.h"
#include "physics/vehicle.h"

static VehicleCatalog g_rosterCatalog;
static bool g_rosterLoaded = false;

static CarRosterRejection g_rejections[CAR_ROSTER_MAX_REJECTIONS];
static int g_rejectionCount = 0;

/* Record why one manifest was refused. Deterministic: entries are appended in catalog order,
 * which vehicle_manifest_load_dir has already sorted by stable id. */
static void record_rejection(const char *id, const char *reason)
{
    if (g_rejectionCount >= CAR_ROSTER_MAX_REJECTIONS) return;
    CarRosterRejection *slot = &g_rejections[g_rejectionCount++];
    snprintf(slot->id, sizeof(slot->id), "%s", id != NULL ? id : "(unnamed)");
    snprintf(slot->reason, sizeof(slot->reason), "%s", reason != NULL ? reason : "");
}

/* The promotion checklist, applied for real. Returns true when every row passes; otherwise
 * records the first failing row's name and evidence, which is the row an author must fix. */
static bool passes_promotion(const VehicleManifest *manifest)
{
    VehiclePromotionReport report;
    if (vehicle_promotion_evaluate(manifest, &report)) return true;
    for (int i = 0; i < report.count; i++) {
        if (report.checks[i].pass) continue;
        char reason[CAR_ROSTER_REJECT_REASON_CHARS];
        snprintf(reason, sizeof(reason), "promotion check '%s' failed: %s",
                 report.checks[i].name, report.checks[i].detail);
        record_rejection(manifest->definition.id, reason);
        return false;
    }
    record_rejection(manifest->definition.id, "promotion evaluation failed");
    return false;
}

/* Every class tag that names a defined class must be satisfied numerically. A tag with no class
 * file is a grouping label only (see the header comment) and constrains nothing. */
static bool passes_class_rules(const VehicleManifest *manifest,
                               const VehicleClassCatalog *classes)
{
    for (int t = 0; t < manifest->classTagCount; t++) {
        const VehicleClass *cls = NULL;
        for (int c = 0; c < classes->count; c++) {
            if (strcmp(classes->items[c].id, manifest->classTags[t]) == 0) {
                cls = &classes->items[c];
                break;
            }
        }
        if (cls == NULL) continue;
        /* Sized so the class id and the framing text still fit alongside the evidence. */
        char detail[CAR_ROSTER_REJECT_REASON_CHARS - VEHICLE_CLASS_ID_CHARS - 32];
        if (!vehicle_class_check_eligibility(cls, manifest, detail, sizeof(detail))) {
            char reason[CAR_ROSTER_REJECT_REASON_CHARS];
            snprintf(reason, sizeof(reason), "class '%s' rejected the car: %s", cls->id,
                     detail);
            record_rejection(manifest->definition.id, reason);
            return false;
        }
    }
    return true;
}

static bool ensure_roster_loaded(void)
{
    if (g_rosterLoaded) return (g_rosterCatalog.count > 0);
    char error[256];
    VehicleCatalog full;
    if (!vehicle_manifest_load_dir("data/vehicles", &full, error, sizeof(error))) {
        return false;
    }
    /* The reviewed class rules, when the build ships them. A missing or unreadable directory
     * leaves an empty catalogue, which makes every class tag unconstrained rather than fatal. */
    VehicleClassCatalog classes;
    char classError[256];
    if (!vehicle_class_load_dir(CAR_ROSTER_CLASS_DIR, &classes, classError,
                                sizeof(classError))) {
        memset(&classes, 0, sizeof(classes));
    }

    /* data/vehicles/ also holds appearance-corpus samples and in-review prototypes that parse
     * fine but must never surface as race cars (issue #31). Compact the loaded catalog down to a
     * gameplay-eligible view: plain structs, so the kept entries are copied in place and the
     * surviving allocation is freed by vehicle_catalog_free like before. */
    g_rejectionCount = 0;
    int kept = 0;
    for (int i = 0; i < full.count; i++) {
        const VehicleManifest *manifest = &full.items[i];
        if (manifest->contentKind != VEHICLE_CONTENT_PLAYER_SELECTABLE) continue;
        if (!passes_promotion(manifest)) continue;
        if (!passes_class_rules(manifest, &classes)) continue;
        if (kept != i) full.items[kept] = full.items[i];
        kept++;
    }
    full.count = kept;
    vehicle_class_catalog_free(&classes);
    g_rosterCatalog = full;
    g_rosterLoaded = true;
    return (g_rosterCatalog.count > 0);
}

void car_roster_reload(void)
{
    if (g_rosterLoaded) {
        vehicle_catalog_free(&g_rosterCatalog);
        g_rosterLoaded = false;
    }
    g_rejectionCount = 0;
}

int car_roster_rejection_count(void)
{
    (void)ensure_roster_loaded();
    return g_rejectionCount;
}

const CarRosterRejection *car_roster_rejection(int index)
{
    if (!ensure_roster_loaded() && g_rejectionCount == 0) return NULL;
    if (index < 0 || index >= g_rejectionCount) return NULL;
    return &g_rejections[index];
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

const VehicleManifest *car_roster_manifest(int index)
{
    if (!ensure_roster_loaded()) return NULL;
    if (index < 0 || index >= g_rosterCatalog.count) return NULL;
    return &g_rosterCatalog.items[index];
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
