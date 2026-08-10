/*
 * car_selection.c — issue #32 headless car-selection model over the player-selectable roster.
 */
#include "game/car_selection.h"

#include <stdio.h>
#include <string.h>

#include "game/car_roster.h"

int car_selection_count(void)
{
    int count = 0;
    int rosterCount = car_roster_count();
    for (int i = 0; i < rosterCount; i++) {
        const VehicleManifest *manifest = car_roster_manifest(i);
        if (manifest != NULL && manifest->eligibleHuman) count++;
    }
    return count;
}

bool car_selection_entry(int index, CarSelectionEntry *out)
{
    if (out == NULL) return false;
    int rosterCount = car_roster_count();
    int seen = 0;
    for (int i = 0; i < rosterCount; i++) {
        const VehicleManifest *manifest = car_roster_manifest(i);
        if (manifest == NULL || !manifest->eligibleHuman) continue;
        if (seen == index) {
            snprintf(out->id, sizeof(out->id), "%s", manifest->definition.id);
            out->manifest = manifest;
            out->rosterIndex = i;
            return true;
        }
        seen++;
    }
    return false;
}

int car_selection_resolve(const char *id)
{
    if (id == NULL) return -1;
    int rosterCount = car_roster_count();
    int selectionIndex = 0;
    for (int i = 0; i < rosterCount; i++) {
        const VehicleManifest *manifest = car_roster_manifest(i);
        if (manifest == NULL || !manifest->eligibleHuman) continue;
        if (strcmp(manifest->definition.id, id) == 0) return selectionIndex;
        selectionIndex++;
    }
    return -1;
}

int car_selection_index_or_default(const char *id)
{
    int index = car_selection_resolve(id);
    return (index >= 0) ? index : 0;
}

bool car_selection_load_recall(char *idBuf, size_t cap)
{
    if (idBuf == NULL || cap == 0) return false;
    FILE *file = fopen(CAR_SELECTION_RECALL_PATH, "r");
    if (file == NULL) return false;
    bool ok = false;
    if (fgets(idBuf, (int)cap, file) != NULL) {
        size_t len = strlen(idBuf);
        /* When the buffer fills exactly, the line may still continue past it: probing the next
         * byte distinguishes "id ended right at the newline/EOF" (complete) from a longer,
         * truncated line, whose prefix we must not validate as if it were the saved id. */
        bool truncated = false;
        if (len > 0 && len == cap - 1 && idBuf[len - 1] != '\n') {
            int next = fgetc(file);
            truncated = (next != EOF && next != '\n');
        }
        /* Trim trailing whitespace, including the CRLF a Windows writer appends. */
        while (len > 0) {
            char c = idBuf[len - 1];
            if (c != '\n' && c != '\r' && c != ' ' && c != '\t') break;
            idBuf[--len] = '\0';
        }
        if (!truncated && len > 0 && vehicle_manifest_id_is_valid(idBuf)) ok = true;
    }
    fclose(file);
    return ok;
}

bool car_selection_save_recall(const char *id)
{
    if (id == NULL || !vehicle_manifest_id_is_valid(id)) return false;
    FILE *file = fopen(CAR_SELECTION_RECALL_PATH, "w");
    if (file == NULL) return false;
    bool ok = (fprintf(file, "%s\n", id) > 0);
    if (fclose(file) != 0) ok = false;
    return ok;
}
