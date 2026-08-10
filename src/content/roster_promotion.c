/*
 * roster_promotion.c — issue #31 promotion checklist evaluation. See the header.
 */
#include "content/roster_promotion.h"

#include <stdio.h>
#include <string.h>

/* Fill one row of the report. `name` is a check literal; `detail` is evidence the author or
 * a failing test reads, so it names values, not just verdicts. */
static void check_set(PromotionCheck *check, const char *name, bool pass, const char *detail)
{
    snprintf(check->name, sizeof(check->name), "%s", name);
    check->pass = pass;
    snprintf(check->detail, sizeof(check->detail), "%s", detail);
}

/* Append one reason to a comma-separated detail, truncating safely. Failing rows list every
 * missing piece rather than stopping at the first, so the report is self-explanatory. */
static void detail_append(char *detail, size_t cap, const char *reason)
{
    const size_t used = strlen(detail);
    if (used == 0) {
        snprintf(detail, cap, "%s", reason);
    } else if (used + 1 < cap) {
        snprintf(detail + used, cap - used, ", %s", reason);
    }
}

/* 1. identity-provenance: stable id, a display name, and recorded provenance. These are the
 * fields a roster entry is shown by and credited to; an unnamed or uncredited car would be
 * unidentifiable in the UI and in the review trail. */
static void check_identity_provenance(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    const bool idOk = vehicle_manifest_id_is_valid(manifest->definition.id);
    const bool nameOk = manifest->displayName[0] != '\0';
    const bool sourceOk = manifest->provenanceSource[0] != '\0';
    const bool authorOk = manifest->provenanceAuthor[0] != '\0';
    if (idOk && nameOk && sourceOk && authorOk) {
        /* Passing evidence quotes the actual values so a reviewer can compare them to the
         * file they are reviewing. Truncation only shortens the tail of the evidence. */
        snprintf(detail, sizeof(detail), "id=%.16s displayName=%.40s source=%.16s author=%.16s",
                 manifest->definition.id, manifest->displayName, manifest->provenanceSource,
                 manifest->provenanceAuthor);
    } else {
        if (!idOk) detail_append(detail, sizeof(detail), "id invalid");
        if (!nameOk) detail_append(detail, sizeof(detail), "displayName empty");
        if (!sourceOk) detail_append(detail, sizeof(detail), "provenance.source empty");
        if (!authorOk) detail_append(detail, sizeof(detail), "provenance.author empty");
    }
    check_set(check, "identity-provenance", idOk && nameOk && sourceOk && authorOk, detail);
}

/* 2. physical-data-complete: a valid spec and a non-zero content hash. The hash is the
 * manifest author's record that the physics were actually written; a zeroed spec could pass
 * a lenient validator but was never content. */
static void check_physical_data(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    const bool specOk = vehicle_spec_is_valid(&manifest->definition.spec);
    const bool hashed = manifest->definition.contentHash != 0;
    if (specOk && hashed) {
        snprintf(detail, sizeof(detail), "spec valid contentHash=0x%08X",
                 manifest->definition.contentHash);
    } else {
        if (!specOk)
            detail_append(detail, sizeof(detail), "vehicle_spec_is_valid rejected the spec");
        if (!hashed) detail_append(detail, sizeof(detail), "contentHash is 0");
    }
    check_set(check, "physical-data-complete", specOk && hashed, detail);
}

/* 3. class-assignment: at least one class tag. The roster UI groups cars by these tags, so
 * an unclassified car would be unfindable; the evidence lists the tags themselves. */
static void check_class_assignment(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    const bool assigned = manifest->classTagCount >= 1;
    if (assigned) {
        /* Evidence is meant to be read, so it must never look complete when it is not. snprintf
         * reports the length it *would* have written, which runs past the buffer as soon as the
         * tag list is long enough; advancing by that number silently drops the tail. Track the
         * bytes actually written instead, and say so with a "..." when the list is cut short. */
        snprintf(detail, sizeof(detail), "classTags: ");
        size_t used = strlen(detail);
        int listed = 0;
        for (int i = 0; i < manifest->classTagCount; i++) {
            char piece[VEHICLE_MANIFEST_CLASS_TAG_CHARS + 2];
            snprintf(piece, sizeof(piece), "%s%s", i > 0 ? ", " : "", manifest->classTags[i]);
            /* Reserve room for the truncation marker so it can always be appended. */
            if (used + strlen(piece) + 4 > sizeof(detail)) break;
            snprintf(detail + used, sizeof(detail) - used, "%s", piece);
            used = strlen(detail);
            listed++;
        }
        if (listed < manifest->classTagCount) {
            snprintf(detail + used, sizeof(detail) - used, "%s...", listed > 0 ? ", " : "");
        }
    } else {
        snprintf(detail, sizeof(detail), "classTagCount=%d (at least 1 required)",
                 manifest->classTagCount);
    }
    check_set(check, "class-assignment", assigned, detail);
}

/* 4. default-setup-valid: the authored baseline setup validates against the definition.
 * Entrants start from this setup, so a rejected baseline would make the car undriveable. */
static void check_default_setup(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    const bool valid = vehicle_setup_is_valid(&manifest->definition, &manifest->defaultSetup);
    if (valid) {
        snprintf(detail, sizeof(detail), "defaultSetup validates against the definition");
    } else {
        snprintf(detail, sizeof(detail), "vehicle_setup_is_valid rejected the default setup");
    }
    check_set(check, "default-setup-valid", valid, detail);
}

/* 5. review-status: the manifest's contentKind records an explicit human review. Corpus
 * samples and unreviewed prototypes exist for tooling, not racing; only a manifest whose
 * kind says "validated" or "player-selectable" may proceed. */
static void check_review_status(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    const bool reviewed = manifest->contentKind == VEHICLE_CONTENT_VALIDATED ||
                          manifest->contentKind == VEHICLE_CONTENT_PLAYER_SELECTABLE;
    if (reviewed) {
        snprintf(detail, sizeof(detail), "contentKind=%s (reviewed)",
                 vehicle_content_kind_name(manifest->contentKind));
    } else {
        snprintf(detail, sizeof(detail), "contentKind=%s is not reviewed content",
                 vehicle_content_kind_name(manifest->contentKind));
    }
    check_set(check, "review-status", reviewed, detail);
}

/* 6. collision-dimensions: the derived collision envelope — bodyHalfWidthM (the collision
 * capsule radius, = width/2, see world/collision.c) and lengthOverallM (wheelbase +
 * overhangs) — inside the ranges the collision system was tuned and validated for. Bounds
 * mirror the dev registry entries the corpus sweeps respect. */
static void check_collision_dimensions(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    const float halfWidthM = manifest->definition.spec.bodyHalfWidthM;
    const float lengthM = manifest->definition.spec.lengthOverallM;
    const bool widthOk = halfWidthM >= 0.4f && halfWidthM <= 1.5f;
    const bool lengthOk = lengthM >= 2.5f && lengthM <= 12.0f;
    if (widthOk && lengthOk) {
        snprintf(detail, sizeof(detail), "collision half-width=%.2fm length=%.2fm", halfWidthM,
                 lengthM);
    } else {
        if (!widthOk) {
            char reason[64];
            snprintf(reason, sizeof(reason), "half-width %.2fm outside [0.4, 1.5]", halfWidthM);
            detail_append(detail, sizeof(detail), reason);
        }
        if (!lengthOk) {
            char reason[64];
            snprintf(reason, sizeof(reason), "length %.2fm outside [2.5, 12]", lengthM);
            detail_append(detail, sizeof(detail), reason);
        }
    }
    check_set(check, "collision-dimensions", widthOk && lengthOk, detail);
}

/* 7. ai-compatibility: AI eligibility on. A promoted car must be drivable by every entrant
 * kind the game ships; a car AI cannot handle would silently narrow a race field. */
static void check_ai_compatibility(const VehicleManifest *manifest, PromotionCheck *check)
{
    char detail[PROMOTION_CHECK_DETAIL_CHARS] = "";
    snprintf(detail, sizeof(detail), "%s",
             manifest->eligibleAi ? "eligibleAi=true" : "eligibleAi=false");
    check_set(check, "ai-compatibility", manifest->eligibleAi, detail);
}

bool vehicle_promotion_evaluate(const VehicleManifest *manifest, VehiclePromotionReport *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (manifest == NULL) return false;

    /* Every row is filled in the fixed checklist order regardless of pass/fail, so one
     * report shows all outstanding work and a test can flip a single check and observe the
     * result flip. */
    check_identity_provenance(manifest, &out->checks[out->count++]);
    check_physical_data(manifest, &out->checks[out->count++]);
    check_class_assignment(manifest, &out->checks[out->count++]);
    check_default_setup(manifest, &out->checks[out->count++]);
    check_review_status(manifest, &out->checks[out->count++]);
    check_collision_dimensions(manifest, &out->checks[out->count++]);
    check_ai_compatibility(manifest, &out->checks[out->count++]);

    for (int i = 0; i < out->count; i++) {
        if (!out->checks[i].pass) return false;
    }
    return true;
}
