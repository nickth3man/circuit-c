/*
 * render_hud.c — screen-space presentation: the diagnostics readout, the arcade HUD, and
 * the full-screen state overlays, plus the palette and text helpers they share. Compiled
 * to an empty translation unit under CIRCUIT_HEADLESS.
 */
#if !defined(CIRCUIT_HEADLESS)

#include "render/render_internal.h"
#include <math.h>
#include "content/vehicle_manifest.h"
#include "game/car_roster.h"
#include "game/car_selection.h"
#include "game/race_presentation.h"
#include "game/race_setup_menu.h"
#include "game/setup_editor.h"
#include "physics/vehicle.h"
#include "core/math_utils.h"
#include "physics/tire.h"
#include "core/units.h"
/* ---- presentation palette, type scale, and screen-space helpers -----------------------
 *
 * Racing HUD palette (documented for the design review):
 *   COL_ACCENT       hot gold    - primary timing, the rpm warning, the car nose marker
 *   COL_ACCENT_WARM  warm orange - NEW BEST flash and other "payoff" moments
 *   COL_COOL         steel cyan  - secondary info and rpm bar fill
 *   COL_TEXT         near-white  - primary HUD text
 *   COL_TEXT_DIM     slate       - secondary and hint text
 *   COL_PANEL        translucent charcoal - backing panels behind HUD clusters
 *   COL_PANEL_EDGE   faint white          - panel outline, separates panel from track
 *   COL_DIM_SCREEN   heavy translucent charcoal - full-screen dim behind overlays
 *
 * Type scale (one scale, used everywhere below):
 *   title 64 | overlay heading 40-48 | results figure 56 | cluster figure 34-46 |
 *   body 18-20 | labels/hints 14-16 | micro 12.
 */
const Color COL_ACCENT = { 255, 198, 64, 255 };
const Color COL_ACCENT_WARM = { 255, 120, 72, 255 };
const Color COL_COOL = { 110, 205, 235, 255 };
const Color COL_TEXT = { 236, 238, 242, 255 };
const Color COL_TEXT_DIM = { 152, 158, 170, 255 };
const Color COL_PANEL = { 12, 14, 18, 170 };
const Color COL_PANEL_EDGE = { 255, 255, 255, 26 };
const Color COL_DIM_SCREEN = { 8, 10, 14, 185 };

/* The car's own palette used to live here as COL_CAR_BODY / COL_CAR_OUTLINE / COL_CAR_CABIN /
 * COL_TIRE / COL_RIM, next to a hardcoded 4.2 x 1.82 m box. Both are gone: the vehicle's
 * colours and its geometry are now derived in src/render/car_visual.c and rasterized by
 * src/render/car_visual_raster.c, and this file only uploads and draws what they produce. Adding a
 * styling decision back here would put it where circuit_tests cannot reach it. */

static void draw_text_centered(const char *text, int y, int fontSize, Color color)
{
    DrawText(text, (SCREEN_W - MeasureText(text, fontSize)) / 2, y, fontSize, color);
}

static void draw_text_centered_shadow(const char *text, int y, int fontSize, Color color)
{
    const int x = (SCREEN_W - MeasureText(text, fontSize)) / 2;
    DrawText(text, x + 3, y + 3, fontSize, (Color){ 0, 0, 0, 170 });
    DrawText(text, x, y, fontSize, color);
}

static void draw_hud_panel(Rectangle rec)
{
    DrawRectangleRounded(rec, 0.16f, 6, COL_PANEL);
    DrawRectangleRoundedLines(rec, 0.16f, 6, COL_PANEL_EDGE);
}

/* Slow pulse between two alpha levels, for "press a key" prompts.
 * Render-only time source; nothing here feeds the simulation. */
static unsigned char pulse_alpha(float cyclesPerSecond, unsigned char lo, unsigned char hi)
{
    const float s = 0.5f + 0.5f * sinf((float)GetTime() * 6.2831853f * cyclesPerSecond);
    return (unsigned char)(lo + (unsigned char)((float)(hi - lo) * s));
}

static const char *gear_label(int selectedGear);
static void format_time(char *out, size_t cap, float seconds);

/* ---- full-screen overlays (STATE_MENU / STATE_PAUSED / STATE_RESULTS) ----------------
 * Pure screen space: called after EndMode2D so the camera transform never touches them.
 * Copy is deliberately short and direct; wording is up for review.
 */
/*
 * The race-setup screen (MAP.md priority 1): the bundle the start path hands to
 * game_configure_session(), one row per control.
 *
 * Labels and values come from the headless model rather than being formatted here, so what the
 * screen says and what a test asserts are the same strings. A row the current config cannot move
 * — the AI rows in a time trial, pit rules on a track without pit geometry — is drawn dim and
 * says so in its value column, because a key that silently does nothing reads as a broken menu.
 *
 * Budgeted against SCREEN_H like the setup overlay below it: 13 rows of 24px from y=292 end at
 * 592, leaving the refusal line and the two hint rows above 700.
 */
static void draw_race_setup(const Game *game)
{
    draw_text_centered_shadow("RACE SETUP", 256, 24, COL_ACCENT);

    const int x = (SCREEN_W - 460) / 2;
    const int valueX = x + 230;
    const int row = 24;
    int y = 292;
    for (int i = 0; i < RACE_SETUP_ROW_COUNT; i++) {
        char label[RACE_SETUP_LABEL_CHARS] = "";
        char value[RACE_SETUP_VALUE_CHARS] = "";
        race_setup_menu_row_label(i, label, sizeof(label));
        race_setup_menu_row_value(&game->raceSetupMenu, &game->raceConfig, i, value,
                                  sizeof(value));
        const bool onCursor = (i == game->raceSetupMenu.cursor);
        const bool active =
            race_setup_menu_row_is_active(&game->raceSetupMenu, &game->raceConfig, i);
        const Color labelColor = onCursor ? COL_ACCENT : (active ? COL_TEXT : COL_TEXT_DIM);
        DrawText(TextFormat("%s%s", onCursor ? "> " : "  ", label), x, y, 18, labelColor);
        DrawText(value, valueX, y, 18, active ? labelColor : COL_TEXT_DIM);
        y += row;
    }

    if (game->startBlockedReason[0] != '\0') {
        draw_text_centered(TextFormat("cannot start: %s", game->startBlockedReason), 608, 16,
                           COL_ACCENT_WARM);
    }
    draw_text_centered(
        "UP/DOWN row    LEFT/RIGHT change    TAB back    P start", 640, 18,
        (Color){ COL_TEXT.r, COL_TEXT.g, COL_TEXT.b, pulse_alpha(0.6f, 90, 255) });
    draw_text_centered("the car above is what races — TAB back, then < / > to change it", 672,
                       14, COL_TEXT_DIM);
}

/*
 * The settings / controls / accessibility screen (MAP.md priority 4).
 *
 * Like the race-setup screen above, every label and value comes from the headless model, so a
 * test asserting what the screen says is asserting the same strings the player reads. The
 * controls tab shows what each action is bound to right now — which is also what the prompts
 * elsewhere show, because both read the same bindings.
 */
static void draw_settings(const Game *game)
{
    const SettingsMenu *menu = &game->settingsMenu;
    draw_text_centered_shadow(TextFormat("SETTINGS — %s", settings_menu_tab_name(menu->tab)),
                              200, 24, COL_ACCENT);

    const int rowCount = settings_menu_row_count(menu);
    /* The controls tab has the most rows, so it sets the layout: two columns keep it above the
     * hint lines whatever the action table grows to. */
    const int kMaxRows = 14;
    const int columns = (rowCount > kMaxRows) ? 2 : 1;
    const int rowsPerColumn = (rowCount + columns - 1) / columns;
    const int marginX = 120;
    const int columnWidth = (SCREEN_W - 2 * marginX) / columns;
    const int top = 244;
    const int rowH = 24;

    for (int i = 0; i < rowCount; i++) {
        char label[SETTINGS_LABEL_CHARS] = "";
        char value[SETTINGS_VALUE_CHARS] = "";
        settings_menu_row_label(menu, i, label, sizeof(label));
        settings_menu_row_value(menu, &game->profile, &game->bindings, i, value, sizeof(value));
        const bool onCursor = (i == menu->cursor);
        const bool capturing = (menu->capturingAction == i);
        const Color c = capturing ? COL_ACCENT_WARM : (onCursor ? COL_ACCENT : COL_TEXT);
        const int column = i / rowsPerColumn;
        const int rowInColumn = i % rowsPerColumn;
        const int x = marginX + column * columnWidth;
        const int y = top + rowInColumn * rowH;
        DrawText(TextFormat("%s%s", onCursor ? "> " : "  ", label), x, y, 17, c);
        DrawText(value, x + columnWidth - 130, y, 17, c);
    }

    const int bottom = top + rowsPerColumn * rowH;
    if (menu->conflictAction >= 0) {
        draw_text_centered(
            TextFormat("that key is already %s", input_action_label(menu->conflictAction)),
            bottom + 10, 16, COL_ACCENT_WARM);
    }
    if (menu->tab == SETTINGS_TAB_CONTROLS) {
        draw_text_centered("UP/DOWN row    LEFT/RIGHT next key    S next tab    O back",
                           bottom + 34, 16, COL_TEXT_DIM);
    } else {
        draw_text_centered("UP/DOWN row    LEFT/RIGHT change    S next tab    O back",
                           bottom + 34, 16, COL_TEXT_DIM);
    }
    draw_text_centered("changes are saved when you leave this screen", bottom + 56, 14,
                       COL_TEXT_DIM);
}

static void draw_overlay_menu(const Game *game)
{
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, COL_DIM_SCREEN);
    draw_text_centered_shadow("CIRCUIT", 92, 56, COL_ACCENT);
    draw_text_centered("a deterministic top-down racing simulator", 158, 18, COL_TEXT_DIM);

    if (game->settingsEditing) {
        /* The settings screen takes the whole menu body: it is not about the car, so the car
         * header below would only be in the way. */
        draw_settings(game);
        return;
    }

    const int selectable = car_selection_count();
    if (selectable == 0 || game->selectedCarIndex < 0) {
        draw_text_centered_shadow("NO SELECTABLE CARS", 280, 32, COL_ACCENT_WARM);
        draw_text_centered(
            "content error — no valid player-selectable vehicle manifest was found", 326, 16,
            COL_TEXT_DIM);
        draw_text_centered("check data/vehicles/*.vehicle.json", 348, 16, COL_TEXT_DIM);
        return;
    }
    CarSelectionEntry entry;
    if (!car_selection_entry(game->selectedCarIndex, &entry)) return;
    const VehicleManifest *m = entry.manifest;
    const VehicleSpec *s = &m->definition.spec;
    draw_text_centered_shadow(m->displayName, 188, 36, COL_TEXT);
    const char *layout = car_roster_layout_name(entry.rosterIndex);
    const char *cls = (m->classTagCount > 0) ? m->classTags[0] : "unclassified";
    draw_text_centered(
        TextFormat("%s   %s   %d / %d", layout, cls, game->selectedCarIndex + 1, selectable),
        232, 18, COL_COOL);
    if (game->raceSetupEditing) {
        /* The race-setup screen takes the body of the menu: the car header above still names
         * what races, and the spec block is what the player traded for the rules. */
        draw_race_setup(game);
        return;
    }
    float peakTorqueNm = 0.0f;
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++) {
        if (s->engineTorqueCurveNm[i] > peakTorqueNm) peakTorqueNm = s->engineTorqueCurveNm[i];
    }
    const int x = (SCREEN_W - 460) / 2;
    int y = 286;
    const int row = 22;
    DrawText(TextFormat("mass            %.0f kg", (double)s->massKg), x, y, 18, COL_TEXT);
    DrawText(TextFormat("peak torque     %.0f N*m", (double)peakTorqueNm), x, y += row, 18,
             COL_TEXT);
    DrawText(TextFormat("gears            %d   final %.2f", s->gearCount,
                        (double)s->finalDriveRatio),
             x, y += row, 18, COL_TEXT);
    DrawText(TextFormat("redline          %.0f rpm", (double)s->engineRedlineRpm), x, y += row,
             18, COL_TEXT);
    DrawText(TextFormat("brake bias       %.2f front", (double)s->brakeBiasFront), x, y += row,
             18, COL_TEXT);
    DrawText(TextFormat("tire grip        F %.2f  R %.2f", (double)s->tireMuLatFront,
                        (double)s->tireMuLatRear),
             x, y += row, 18, COL_TEXT);
    DrawText(
        TextFormat("drag             Cd %.2f", (double)vehicle_effective_drag_coefficient(s)),
        x, y += row, 18, COL_TEXT);
    DrawText(TextFormat("tires            %d/%dR%d", (int)s->tireSectionWidthFrontMm,
                        (int)s->tireAspectFrontPct, (int)s->tireRimDiameterFrontIn),
             x, y += row, 18, COL_TEXT_DIM);
    /*
     * Everything below is laid out against SCREEN_H (720) with the bottom prompt as the last
     * row. The window is a fixed size, so a y past 720 is not "further down the page" — it is
     * simply not drawn, which is how the control hints and the tail of the setup list went
     * missing. Both blocks are budgeted to end above 700.
     */
    if (game->setupEditing) {
        /* The description is dropped here rather than reflowed: the setup list needs the room,
         * and the car it belongs to is still named at the top of the screen. */
        const SetupEditor *ed = &game->setupEditor;
        draw_text_centered_shadow("SETUP", 462, 24, COL_ACCENT);
        /*
         * Every item the cursor can reach is on screen, whatever the editor holds. The cursor
         * wraps across all itemCount entries, so a fixed row budget would land the highlight on
         * an invisible row while UP/DOWN edited a value the player could not see.
         *
         * The vertical budget is the fixed quantity — rows must end above the hint and
         * validity lines, which must themselves end above SCREEN_H. So rows are capped first
         * and the column count is whatever that requires, up to SETUP_EDITOR_MAX_ITEMS. Today
         * that is 12 items in 2 columns; a full 24 would lay out in 4 without clipping.
         */
        enum {
            kSetupTop = 498,
            kSetupRow = 22,
            kSetupRowLimit = 648, /* last row's baseline; hints and warning go below */
            kSetupMaxRows = (kSetupRowLimit - kSetupTop) / kSetupRow,
            kSetupMarginX = 60,
        };
        /* Asserted rather than clamped at draw time: these are fixed layout constants, so a
         * budget too small for a row — or an editor capacity needing more columns than the
         * width can carry — is a build-time mistake, not something to paper over per frame. */
        _Static_assert(kSetupMaxRows >= 1, "the setup row budget must fit at least one row");
        _Static_assert(SETUP_EDITOR_MAX_ITEMS <= kSetupMaxRows * 4,
                       "the setup overlay lays out at most four columns");
        const int setupTop = kSetupTop;
        const int setupRow = kSetupRow;
        const int columns =
            (ed->itemCount > 0) ? ((ed->itemCount + kSetupMaxRows - 1) / kSetupMaxRows) : 1;
        const int rowsPerColumn =
            (ed->itemCount > 0) ? ((ed->itemCount + columns - 1) / columns) : 0;
        const int marginX = kSetupMarginX;
        const int columnWidth = (SCREEN_W - 2 * marginX) / columns;
        for (int i = 0; i < ed->itemCount; i++) {
            const SetupEditorItem *it = &ed->items[i];
            const float val = setup_editor_value(ed, i);
            const bool onCursor = (i == game->setupCursor);
            const Color c = onCursor ? COL_ACCENT : COL_TEXT;
            const int column = (rowsPerColumn > 0) ? (i / rowsPerColumn) : 0;
            const int rowInColumn = (rowsPerColumn > 0) ? (i % rowsPerColumn) : i;
            DrawText(TextFormat("%s%s  %g %s", onCursor ? "> " : "  ", it->key, (double)val,
                                it->unit),
                     marginX + column * columnWidth, setupTop + rowInColumn * setupRow, 16, c);
        }
        const int setupBottom = setupTop + rowsPerColumn * setupRow;
        draw_text_centered("LEFT/RIGHT item    UP/DOWN adjust    D reset    S back",
                           setupBottom + 12, 14, COL_TEXT_DIM);
        if (!setup_editor_is_valid(ed)) {
            draw_text_centered("setup out of bounds — adjust to continue", setupBottom + 34, 14,
                               COL_ACCENT_WARM);
        }
        return;
    }
    if (m->description[0] != '\0') {
        draw_text_centered(m->description, y + row + 16, 15, COL_TEXT_DIM);
    }
    if (game->startBlockedReason[0] != '\0') {
        draw_text_centered(TextFormat("cannot start: %s", game->startBlockedReason), 612, 16,
                           COL_ACCENT_WARM);
    }
    draw_text_centered(
        "< / > car    S setup    TAB race setup    O settings    P start", 640, 18,
        (Color){ COL_TEXT.r, COL_TEXT.g, COL_TEXT.b, pulse_alpha(0.6f, 90, 255) });
    {
        const InputBindings *b = &game->bindings;
        draw_text_centered(
            TextFormat(
                "%s/%s throttle & brake    %s/%s steer    %s handbrake    %s/%s shift    "
                "%s pause    %s reset",
                input_bindings_label(b, INPUT_ACTION_THROTTLE),
                input_bindings_label(b, INPUT_ACTION_BRAKE),
                input_bindings_label(b, INPUT_ACTION_STEER_LEFT),
                input_bindings_label(b, INPUT_ACTION_STEER_RIGHT),
                input_bindings_label(b, INPUT_ACTION_HANDBRAKE),
                input_bindings_label(b, INPUT_ACTION_SHIFT_DOWN),
                input_bindings_label(b, INPUT_ACTION_SHIFT_UP),
                input_bindings_label(b, INPUT_ACTION_PAUSE),
                input_bindings_label(b, INPUT_ACTION_RESET)),
            672, 14, COL_TEXT_DIM);
    }
}

static void draw_overlay_paused(const Game *game)
{
    (void)game;
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H,
                  (Color){ COL_DIM_SCREEN.r, COL_DIM_SCREEN.g, COL_DIM_SCREEN.b, 150 });
    draw_text_centered_shadow("PAUSED", 300, 48, COL_TEXT);
    draw_text_centered("P resume    R reset", 372, 18, COL_TEXT_DIM);
}

/*
 * The classified results (MAP.md priority 3).
 *
 * Every value on this screen is read from `session.results`, the immutable snapshot the session
 * wrote once on entry to RACE_PHASE_CLASSIFIED. That is the whole point of the change: the
 * session had been building a full classification for some time and the screen was showing one
 * lap time, which is MAP.md's known issue #5. Nothing is recomputed here — not the order, not
 * the gaps, not who set the fastest lap.
 *
 * A session that ends without valid results (abandoned before classification, or a run that
 * never reached the session at all) falls back to the single lap time, because there genuinely
 * is no classification to show and inventing an empty table would be worse.
 */
static void draw_overlay_results(const Game *game)
{
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, COL_DIM_SCREEN);

    const RaceResults *results = &game->session.results;
    if (!results->valid || results->count <= 0) {
        draw_text_centered_shadow("RUN COMPLETE", 196, 40, COL_TEXT);
        char lap[32];
        format_time(lap, sizeof(lap), game->progress.lastLapTimeS);
        draw_text_centered_shadow(lap, 286, 56, COL_ACCENT);
        draw_text_centered("LAP TIME", 352, 16, COL_TEXT_DIM);
        draw_text_centered("P drive again    R menu", 504, 18, COL_TEXT_DIM);
        return;
    }

    draw_text_centered_shadow("CLASSIFICATION", 60, 40, COL_TEXT);

    /* Column layout. The table is budgeted against SCREEN_H: a header at 132, up to
     * RACE_MAX_ENTRANTS rows of 30px from 158 ends at 398, leaving the summary and prompts
     * above 700. */
    enum {
        kTableX = 150,
        kRowTop = 158,
        kRowH = 30,
        kColPos = 0,
        kColCar = 46,
        kColLaps = 250,
        kColTime = 330,
        kColBest = 500,
        kColStatus = 670,
        kColPen = 800,
    };
    const int headerY = 132;
    DrawText("POS", kTableX + kColPos, headerY, 14, COL_TEXT_DIM);
    DrawText("CAR", kTableX + kColCar, headerY, 14, COL_TEXT_DIM);
    DrawText("LAPS", kTableX + kColLaps, headerY, 14, COL_TEXT_DIM);
    DrawText("TIME / GAP", kTableX + kColTime, headerY, 14, COL_TEXT_DIM);
    DrawText("BEST LAP", kTableX + kColBest, headerY, 14, COL_TEXT_DIM);
    DrawText("STATUS", kTableX + kColStatus, headerY, 14, COL_TEXT_DIM);
    DrawText("PEN", kTableX + kColPen, headerY, 14, COL_TEXT_DIM);

    const EntrantId localId = game->session.roster.localEntrantId;
    for (int i = 0; i < results->count && i < RACE_MAX_ENTRANTS; i++) {
        const RaceResultRow *row = &results->rows[i];
        const int y = kRowTop + kRowH * i;
        const bool isLocal = (row->entrantId == localId);
        const bool placed = (row->status == RACE_STATUS_FINISHED);
        const Color c = isLocal ? COL_ACCENT : (placed ? COL_TEXT : COL_TEXT_DIM);

        if (row->finishPosition > 0) {
            DrawText(TextFormat("%d", row->finishPosition), kTableX + kColPos, y, 18, c);
        } else {
            DrawText("-", kTableX + kColPos, y, 18, c);
        }
        DrawText(row->carId, kTableX + kColCar, y, 18, c);
        DrawText(TextFormat("%d", row->lapsCompleted), kTableX + kColLaps, y, 18, c);

        /* The winner shows an elapsed time; everyone else who finished shows the gap to it.
         * A row with no finish time shows nothing rather than 0:00.00. */
        char timeText[32];
        if (row->status != RACE_STATUS_FINISHED) {
            (void)snprintf(timeText, sizeof(timeText), "%s", "—");
        } else if (row->finishPosition == 1) {
            format_time(timeText, sizeof(timeText), row->finishTimeS);
        } else {
            (void)snprintf(timeText, sizeof(timeText), "+%.2fs", (double)row->gapToLeaderS);
        }
        DrawText(timeText, kTableX + kColTime, y, 18, c);

        char bestText[32];
        format_time(bestText, sizeof(bestText), row->bestLapTimeS);
        const bool ownsFastest = (results->fastestLapEntrantId != RACE_ENTRANT_ID_NONE) &&
                                 (results->fastestLapEntrantId == row->entrantId);
        DrawText(bestText, kTableX + kColBest, y, 18, ownsFastest ? COL_ACCENT_WARM : c);

        DrawText(race_status_name(row->status), kTableX + kColStatus, y, 16,
                 placed ? c : COL_ACCENT_WARM);

        if (row->penaltyCount > 0) {
            DrawText(TextFormat("%d (+%.1fs)", row->penaltyCount, (double)row->penaltyTimeS),
                     kTableX + kColPen, y, 14, COL_ACCENT_WARM);
        }
    }

    /* Session summary under the table. */
    const int summaryY = kRowTop + kRowH * results->count + 24;
    if (results->fastestLapTimeS > 0.0f) {
        const char *holder = "";
        for (int i = 0; i < results->count; i++) {
            if (results->rows[i].entrantId == results->fastestLapEntrantId)
                holder = results->rows[i].carId;
        }
        char fastest[32];
        format_time(fastest, sizeof(fastest), results->fastestLapTimeS);
        draw_text_centered(TextFormat("fastest lap  %s  %s", fastest, holder), summaryY, 18,
                           COL_ACCENT_WARM);
    }

    draw_text_centered(
        "P race again    R menu", 640, 18,
        (Color){ COL_TEXT.r, COL_TEXT.g, COL_TEXT.b, pulse_alpha(0.6f, 90, 255) });
}

/* Format a lap or gap time as m:ss.hh into `out`. A non-positive time has not been set. */
static void format_time(char *out, size_t cap, float seconds)
{
    if (seconds <= 0.0f) {
        (void)snprintf(out, cap, "--:--.--");
        return;
    }
    const int minutes = (int)(seconds / 60.0f);
    const float rest = seconds - (float)minutes * 60.0f;
    (void)snprintf(out, cap, "%d:%05.2f", minutes, (double)rest);
}

/* A labelled horizontal meter, filled left to right. `warnAbove` flips the fill to the warm
 * accent — used for the readouts where a high number is the bad news (wear, damage). */
static void draw_meter(Rectangle bar, float fraction, bool warn)
{
    const float f = clampf(fraction, 0.0f, 1.0f);
    DrawRectangleRec(bar, (Color){ 255, 255, 255, 22 });
    DrawRectangleRec((Rectangle){ bar.x, bar.y, bar.width * f, bar.height },
                     warn ? COL_ACCENT_WARM : COL_COOL);
}

static const char *pit_state_label(PitState state)
{
    switch (state) {
        case PIT_STATE_ENTERING: return "PIT ENTRY";
        case PIT_STATE_IN_LANE: return "PIT LANE — LIMITER";
        case PIT_STATE_AT_BOX: return "IN THE BOX";
        case PIT_STATE_EXITING: return "PIT EXIT";
        case PIT_STATE_NONE:
        case PIT_STATE_COUNT:
        default: return NULL;
    }
}

/* ---- arcade HUD clusters -------------------------------------------------------------
 *
 * EVERY RACE NUMBER ON THIS SCREEN COMES FROM ONE SNAPSHOT. `race_presentation_snapshot()` is
 * the authoritative read of the session at this tick — position, order, gaps, laps, fastest lap,
 * penalties, wrong-way, pit state. The HUD computes nothing from it: that is what stops the
 * screen and the results from disagreeing about the same race, and it is why MAP.md's priority 2
 * is "connect the renderer to the snapshot" rather than "add readouts".
 *
 * The vehicle cluster is the exception and deliberately so: speed, gear and rpm are properties
 * of the car in the player's hands, not of the classification, and they are read from the local
 * entrant's own instance.
 *
 * Clusters, and what each answers:
 *   speed   bottom-left   how fast am I going          (car authority)
 *   lap     top-center    how far through am I         (snapshot)
 *   order   top-right     who is where, and by how much(snapshot)
 *   car     bottom-right  what shape is the car in     (snapshot)
 *   status  center        what is the race doing to me (snapshot)
 */
void render_hud_draw_arcade(const Game *game)
{
    RacePresentationSnapshot snap;
    race_presentation_snapshot(&game->session, &game->trackDef, &snap);
    const RaceEntrant *local = race_roster_local_const(&game->session.roster);
    if (local == NULL) return;
    const VehicleInstance *inst = &local->instance;

    /* ---- speed cluster (bottom-left) ---- */
    {
        const Rectangle panel = { 18.0f, SCREEN_H - 168.0f, 244.0f, 140.0f };
        draw_hud_panel(panel);

        const float kmh = inst->derived.speedMps * 3.6f;
        const char *kmhText = TextFormat("%.0f", (double)kmh);
        DrawText(kmhText, (int)panel.x + 16, (int)panel.y + 12, 46, COL_TEXT);
        DrawText("KM/H", (int)panel.x + 20 + MeasureText(kmhText, 46), (int)panel.y + 40, 16,
                 COL_TEXT_DIM);

        const char *modeLabel = inst->autoTrans.enabled ? "AUTO " : "";
        DrawText(TextFormat("%sGEAR %s", modeLabel, gear_label(inst->vehicle.selectedGear)),
                 (int)panel.x + 16, (int)panel.y + 66, 18, COL_TEXT);

        /* RPM bar: cool cyan, flipping to accent gold near the redline. */
        const float idleRpm = inst->spec.engineIdleRpm;
        const float redlineRpm = inst->spec.engineRedlineRpm;
        const float rpmFrac =
            clampf((inst->vehicle.engineRpm - idleRpm) / (redlineRpm - idleRpm), 0.0f, 1.0f);
        const Rectangle barBg = { panel.x + 16.0f, panel.y + 98.0f, panel.width - 32.0f,
                                  10.0f };
        DrawRectangleRec(barBg, (Color){ 255, 255, 255, 22 });
        DrawRectangleRec((Rectangle){ barBg.x, barBg.y, barBg.width * rpmFrac, barBg.height },
                         (rpmFrac > 0.85f) ? COL_ACCENT : COL_COOL);
        DrawText(TextFormat("%.0f RPM", (double)inst->vehicle.engineRpm), (int)panel.x + 16,
                 (int)panel.y + 114, 12, COL_TEXT_DIM);
    }

    /* ---- lap cluster (top-center) ---- */
    {
        const Rectangle panel = { (SCREEN_W - 360.0f) * 0.5f, 16.0f, 360.0f, 82.0f };
        draw_hud_panel(panel);

        const int targetLaps = (snap.targetLaps > 0) ? snap.targetLaps : RESULTS_TARGET_LAPS;
        int shownLap = snap.localLapsCompleted + 1;
        if (shownLap > targetLaps) shownLap = targetLaps;
        DrawText(TextFormat("LAP %d/%d", shownLap, targetLaps), (int)panel.x + 16,
                 (int)panel.y + 14, 18, COL_TEXT);

        char timer[32];
        format_time(timer, sizeof(timer), snap.localLapTimerS);
        DrawText(timer, (int)(panel.x + (panel.width - (float)MeasureText(timer, 22)) * 0.5f),
                 (int)panel.y + 12, 22, COL_TEXT);

        /* Position is the snapshot's, not a count of who is in front: live order is the
         * session's job and a second opinion here would eventually be a different one. */
        if (snap.localPosition > 0) {
            const char *posText = TextFormat("P%d/%d", snap.localPosition, snap.entrantCount);
            DrawText(posText, (int)(panel.x + panel.width) - 16 - MeasureText(posText, 20),
                     (int)panel.y + 13, 20, COL_ACCENT);
        }

        char best[32];
        char fastest[32];
        format_time(best, sizeof(best), snap.localBestLapTimeS);
        format_time(fastest, sizeof(fastest), snap.fastestLapTimeS);
        const bool ownsFastest = (snap.fastestLapEntrantId != RACE_ENTRANT_ID_NONE) &&
                                 (snap.fastestLapEntrantId == local->id);
        DrawText(TextFormat("BEST %s", best), (int)panel.x + 16, (int)panel.y + 52, 14,
                 COL_TEXT_DIM);
        DrawText(TextFormat("SESSION %s", fastest),
                 (int)(panel.x + panel.width) - 16 -
                     MeasureText(TextFormat("SESSION %s", fastest), 14),
                 (int)panel.y + 52, 14, ownsFastest ? COL_ACCENT_WARM : COL_TEXT_DIM);
    }

    /* ---- order cluster (top-right) ----
     * Every entrant, in the session's live order, with the time gap to the leader. Skipped for
     * a solo session, where a running order of one is noise. */
    if (snap.entrantCount > 1) {
        const float rowH = 20.0f;
        const Rectangle panel = { SCREEN_W - 296.0f, 16.0f, 278.0f,
                                  30.0f + rowH * (float)snap.entrantCount };
        draw_hud_panel(panel);
        DrawText("ORDER", (int)panel.x + 14, (int)panel.y + 8, 14, COL_TEXT_DIM);
        for (int i = 0; i < snap.entrantCount; i++) {
            const PresentationEntrantRow *row = &snap.rows[i];
            const Color c = row->isLocal ? COL_ACCENT : COL_TEXT;
            const int y = (int)panel.y + 26 + (int)(rowH * (float)i);
            DrawText(TextFormat("%d", row->livePosition), (int)panel.x + 14, y, 16, c);
            DrawText(row->carId, (int)panel.x + 40, y, 16, c);
            /* Leader shows laps rather than a gap to itself; a car with no usable speed shows
             * the distance it is behind, because a time gap would be a fiction. */
            const char *right;
            if (i == 0) {
                right = TextFormat("L%d", row->lapsCompleted);
            } else if (row->gapToLeaderS > 0.0f) {
                right = TextFormat("+%.1fs", (double)row->gapToLeaderS);
            } else if (row->distanceToLeaderM > 0.0f) {
                right = TextFormat("+%.0fm", (double)row->distanceToLeaderM);
            } else {
                right = "--";
            }
            DrawText(right, (int)(panel.x + panel.width) - 14 - MeasureText(right, 16), y, 16,
                     row->isLocal ? COL_ACCENT : COL_TEXT_DIM);
        }
    }

    /* ---- car cluster (bottom-right) ---- */
    {
        const Rectangle panel = { SCREEN_W - 262.0f, SCREEN_H - 168.0f, 244.0f, 140.0f };
        draw_hud_panel(panel);
        const int x = (int)panel.x + 16;
        const float barW = panel.width - 32.0f;
        int y = (int)panel.y + 12;

        DrawText(TextFormat("FUEL   %3.0f%%", (double)snap.fuelPercent), x, y, 14, COL_TEXT);
        draw_meter((Rectangle){ panel.x + 16.0f, (float)y + 18.0f, barW, 8.0f },
                   snap.fuelPercent / 100.0f, snap.fuelPercent < 15.0f);
        y += 36;
        DrawText(TextFormat("TYRES  %3.0f%%", (double)snap.tireWearPercent), x, y, 14,
                 COL_TEXT);
        draw_meter((Rectangle){ panel.x + 16.0f, (float)y + 18.0f, barW, 8.0f },
                   snap.tireWearPercent / 100.0f, snap.tireWearPercent > 60.0f);
        y += 36;
        DrawText(TextFormat("DAMAGE %3.0f%%", (double)snap.damagePercent), x, y, 14, COL_TEXT);
        draw_meter((Rectangle){ panel.x + 16.0f, (float)y + 18.0f, barW, 8.0f },
                   snap.damagePercent / 100.0f, snap.damagePercent > 25.0f);
        y += 34;
        DrawText(TextFormat("ABS %d   TCS %d", snap.absLevel, snap.tcsLevel), x, y, 12,
                 COL_TEXT_DIM);
    }

    /* ---- status band (center) ----
     * One line at a time, most urgent first. A countdown owns the screen while it runs; after
     * that the band is for the things the race is doing to the player. */
    {
        if (snap.phase == RACE_PHASE_COUNTDOWN) {
            /* Whole seconds remaining, rounded up, so the last tick of "1" is still "1". */
            const int ticks = snap.countdownTicksRemaining;
            const int secondsLeft = (int)(((float)ticks * FIXED_DT_S) + 0.999f);
            const char *text = (secondsLeft > 0) ? TextFormat("%d", secondsLeft) : "GO";
            draw_text_centered_shadow(text, 200, 96, COL_ACCENT);
            draw_text_centered("hold the grid", 306, 16, COL_TEXT_DIM);
        } else if (snap.wrongWay) {
            draw_text_centered_shadow("WRONG WAY", 200, 40, COL_ACCENT_WARM);
        } else {
            const char *pit = pit_state_label(snap.localPitState);
            if (pit != NULL) {
                draw_text_centered_shadow(pit, 200, 30, COL_ACCENT);
                if (snap.pitServiceRemainingS > 0.0f) {
                    draw_text_centered(TextFormat("service %.1fs   box %d",
                                                  (double)snap.pitServiceRemainingS,
                                                  snap.pitAssignedBox),
                                       238, 16, COL_TEXT_DIM);
                }
            }
        }
        if (snap.pendingPenalties > 0) {
            draw_text_centered(TextFormat("%d PENALTY%s PENDING", snap.pendingPenalties,
                                          snap.pendingPenalties == 1 ? "" : "S"),
                               SCREEN_H - 200, 18, COL_ACCENT_WARM);
        }
    }

    /* Hot-reload notice, top-left: preserves the information the old HUD line carried. */
    if (game->reloadFlashTimerS > 0.0f) {
        DrawText("module reloaded - state preserved", 18, 18, 14, COL_ACCENT);
    }

    /* The control hints name the keys that are actually bound, not the ones that shipped. A
     * player who rebinds throttle and is still told to press W has been given a screen that
     * lies to them, which is the second half of MAP.md priority 4. F2 is the physics lab's own
     * key and is not a bindable action, so it stays literal. */
    {
        const InputBindings *b = &game->bindings;
        const char *hint = TextFormat(
            "%s throttle  %s brake  %s handbrake  %s/%s shift  %s/%s steer  %s reset  "
            "%s diagnostics  F2 physics lab",
            input_bindings_label(b, INPUT_ACTION_THROTTLE),
            input_bindings_label(b, INPUT_ACTION_BRAKE),
            input_bindings_label(b, INPUT_ACTION_HANDBRAKE),
            input_bindings_label(b, INPUT_ACTION_SHIFT_DOWN),
            input_bindings_label(b, INPUT_ACTION_SHIFT_UP),
            input_bindings_label(b, INPUT_ACTION_STEER_LEFT),
            input_bindings_label(b, INPUT_ACTION_STEER_RIGHT),
            input_bindings_label(b, INPUT_ACTION_RESET),
            input_bindings_label(b, INPUT_ACTION_DEBUG));
        DrawText(hint, (SCREEN_W - MeasureText(hint, 14)) / 2, SCREEN_H - 26, 14, COL_TEXT_DIM);
    }
}

static void hud_line(int x, int *y, const char *text, Color color)
{
    DrawText(text, x, *y, 16, color);
    *y += 19;
}

static Vector2 plot_point(Rectangle bounds, float x, float xMin, float xMax, float y,
                          float yMin, float yMax)
{
    return (Vector2){ bounds.x + (x - xMin) / (xMax - xMin) * bounds.width,
                      bounds.y + bounds.height - (y - yMin) / (yMax - yMin) * bounds.height };
}

static void draw_curve_axes(Rectangle bounds, float xMin, float xMax, float yMin, float yMax)
{
    DrawRectangleRec(bounds, (Color){ 16, 18, 22, 235 });
    DrawRectangleLinesEx(bounds, 1.0f, (Color){ 100, 106, 116, 255 });
    DrawLineV(plot_point(bounds, xMin, xMin, xMax, 0.0f, yMin, yMax),
              plot_point(bounds, xMax, xMin, xMax, 0.0f, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
    DrawLineV(plot_point(bounds, 0.0f, xMin, xMax, yMin, yMin, yMax),
              plot_point(bounds, 0.0f, xMin, xMax, yMax, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
}

static void draw_tire_curve_panel(const Game *game)
{
    const Rectangle lateral = { SCREEN_W - 390.0f, 26.0f, 365.0f, 145.0f };
    const Rectangle longitudinal = { SCREEN_W - 390.0f, 205.0f, 365.0f, 145.0f };
    const float latMin = -0.55f;
    const float latMax = 0.55f;
    const float forceMin = -1.4f;
    const float forceMax = 1.4f;
    draw_curve_axes(lateral, latMin, latMax, forceMin, forceMax);
    DrawText("LATERAL  normalized force / wheel load", (int)lateral.x, (int)lateral.y - 19, 14,
             RAYWHITE);
    DrawText("front", (int)lateral.x + 6, (int)lateral.y + 5, 12, ORANGE);
    DrawText("rear", (int)lateral.x + 52, (int)lateral.y + 5, 12, SKYBLUE);
    Vector2 prevFront = { 0.0f, 0.0f };
    Vector2 prevRear = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(latMin, latMax, (float)i / 120.0f);
        const float front =
            -game->spec.tireMuLatFront *
            tire_normalized_curve(game->spec.tireBLatFront, game->spec.tireCLatFront, slip);
        const float rear =
            -game->spec.tireMuLatRear *
            tire_normalized_curve(game->spec.tireBLatRear, game->spec.tireCLatRear, slip);
        const Vector2 pFront =
            plot_point(lateral, slip, latMin, latMax, front, forceMin, forceMax);
        const Vector2 pRear =
            plot_point(lateral, slip, latMin, latMax, rear, forceMin, forceMax);
        if (i > 0) {
            DrawLineV(prevFront, pFront, ORANGE);
            DrawLineV(prevRear, pRear, SKYBLUE);
        }
        prevFront = pFront;
        prevRear = pRear;
    }
    const float currentFront =
        -game->spec.tireMuLatFront * tire_normalized_curve(game->spec.tireBLatFront,
                                                           game->spec.tireCLatFront,
                                                           game->derived.frontSlipAngleRad);
    const float currentRear =
        -game->spec.tireMuLatRear * tire_normalized_curve(game->spec.tireBLatRear,
                                                          game->spec.tireCLatRear,
                                                          game->derived.rearSlipAngleRad);
    DrawCircleV(plot_point(lateral, game->derived.frontSlipAngleRad, latMin, latMax,
                           currentFront, forceMin, forceMax),
                4.0f, ORANGE);
    DrawCircleV(plot_point(lateral, game->derived.rearSlipAngleRad, latMin, latMax, currentRear,
                           forceMin, forceMax),
                4.0f, SKYBLUE);

    const float longMin = -1.25f;
    const float longMax = 1.25f;
    draw_curve_axes(longitudinal, longMin, longMax, -1.1f, 1.1f);
    DrawText("LONGITUDINAL  normalized force / wheel load", (int)longitudinal.x,
             (int)longitudinal.y - 19, 14, RAYWHITE);
    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(longMin, longMax, (float)i / 120.0f);
        const float force =
            game->spec.tireMuLongScale *
            tire_normalized_curve(game->spec.tireBLong, game->spec.tireCLong, slip);
        const Vector2 point =
            plot_point(longitudinal, slip, longMin, longMax, force, -1.1f, 1.1f);
        if (i > 0) DrawLineV(previous, point, LIME);
        previous = point;
    }
    const float rearSlip = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
    const float rearLong =
        game->spec.tireMuLongScale *
        tire_normalized_curve(game->spec.tireBLong, game->spec.tireCLong, rearSlip);
    DrawCircleV(plot_point(longitudinal, rearSlip, longMin, longMax, rearLong, -1.1f, 1.1f),
                4.0f, YELLOW);
    DrawText("zero axes; curve peaks are the configured friction references",
             (int)longitudinal.x, (int)longitudinal.y + (int)longitudinal.height + 5, 11,
             (Color){ 155, 160, 170, 255 });
}

static const char *gear_label(int selectedGear)
{
    if (selectedGear < 0) return "R";
    if (selectedGear == 0) return "N";
    static const char *const labels[MAX_GEARS] = { "1", "2", "3", "4", "5", "6", "7", "8" };
    if (selectedGear <= MAX_GEARS) return labels[selectedGear - 1];
    return "?";
}

/* ---- raw physics diagnostics (F1) -------------------------------------------------
 * The development readout. Everything here draws only when the debug overlay is on; the
 * always-on presentation is the arcade HUD. Lines are unchanged from the previous
 * always-on stack, just gated. */
void render_hud_draw_diagnostics(const Game *game, float alpha)
{
    if (game->debugOverlay) {
        const Color label = (Color){ 235, 235, 235, 255 };
        const Color dim = (Color){ 155, 160, 170, 255 };
        int y = 12;
        hud_line(14, &y, "CIRCUIT diagnostics", label);
        hud_line(14, &y,
                 TextFormat("pos (%+.2f,%+.2f) m  heading %+.3f rad",
                            (double)game->vehicle.positionM.x,
                            (double)game->vehicle.positionM.y,
                            (double)game->vehicle.headingRad),
                 dim);
        hud_line(14, &y,
                 TextFormat("vx %+.3f m/s  vy %+.3f m/s  speed %.3f m/s  yaw %+.3f rad/s",
                            (double)game->vehicle.velocityLongitudinalMps,
                            (double)game->vehicle.velocityLateralMps,
                            (double)game->derived.speedMps, (double)game->vehicle.yawRateRadS),
                 label);
        hud_line(14, &y,
                 TextFormat("steer %+.3f  slip F/R %+.3f / %+.3f rad  sideslip %+.3f",
                            (double)game->vehicle.frontRoadWheelAngleRad,
                            (double)game->derived.frontSlipAngleRad,
                            (double)game->derived.rearSlipAngleRad,
                            (double)game->derived.bodySideslipRad),
                 dim);
        hud_line(14, &y,
                 TextFormat("gear %s  engine %.0f rpm  rear drive %+.1f Nm",
                            gear_label(game->vehicle.selectedGear),
                            (double)game->vehicle.engineRpm,
                            (double)game->derived.drivelineTorqueNm),
                 label);
        hud_line(14, &y,
                 TextFormat("omega FL/FR/R %.2f / %.2f / %.2f rad/s  surface R %+.2f m/s",
                            (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS,
                            (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS,
                            (double)game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS,
                            (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS *
                                     vehicle_wheel_radius_m(&game->spec, WHEEL_REAR_LEFT))),
                 dim);
        hud_line(14, &y,
                 TextFormat("kappa FL/FR/RL/RR %+.3f %+.3f %+.3f %+.3f",
                            (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio,
                            (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio,
                            (double)game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio,
                            (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio),
                 dim);
        hud_line(14, &y,
                 TextFormat(
                     "load F/R %.1f / %.1f N  transfer %+.1f N  lateral F/R %+.1f / %+.1f N",
                     (double)game->derived.normalLoadFrontN,
                     (double)game->derived.normalLoadRearN, (double)game->derived.loadTransferN,
                     (double)game->derived.frontLateralForceN,
                     (double)game->derived.rearLateralForceN),
                 dim);
        hud_line(14, &y,
                 TextFormat("body force (%+.1f,%+.1f) N  yaw torque %+.1f Nm  blend %.3f",
                            (double)game->derived.totalBodyForceN.x,
                            (double)game->derived.totalBodyForceN.y,
                            (double)game->derived.totalYawTorqueNm,
                            (double)game->derived.lowSpeedBlend),
                 dim);
        hud_line(14, &y,
                 TextFormat("substeps %d  backlog %d  alpha %.3f  tick %llu  checksum %08x",
                            game->lastSubstepCount, game->physicsBacklogDrops, (double)alpha,
                            (unsigned long long)game->sim.tick, game->stateChecksum),
                 label);
        hud_line(14, &y,
                 TextFormat("reloads %d%s  gear %s  render %.1f px/m", game->reloadCount,
                            game->reloadFlashTimerS > 0.0f ? " (state preserved)" : "",
                            gear_label(game->vehicle.selectedGear),
                            (double)game->renderPixelsPerMeter),
                 dim);
        hud_line(14, &y,
                 TextFormat("Lap: %d  Timer: %d:%05.2f  Checkpoint: %d / %d",
                            game->progress.lap, (int)(game->progress.lapTimerS / 60.0f),
                            (double)(game->progress.lapTimerS -
                                     (float)(int)(game->progress.lapTimerS / 60.0f) * 60.0f),
                            game->progress.nextCheckpoint, game->trackDef.checkpointCount),
                 label);
        {
            hud_line(
                14, &y,
                TextFormat("pure Fx F/R %+.0f/%+.0f  pure Fy F/R %+.0f/%+.0f N",
                           (double)(game->derived.pureLongitudinalForceN[WHEEL_FRONT_LEFT] +
                                    game->derived.pureLongitudinalForceN[WHEEL_FRONT_RIGHT]),
                           (double)(game->derived.pureLongitudinalForceN[WHEEL_REAR_LEFT] +
                                    game->derived.pureLongitudinalForceN[WHEEL_REAR_RIGHT]),
                           (double)(game->derived.pureLateralForceN[WHEEL_FRONT_LEFT] +
                                    game->derived.pureLateralForceN[WHEEL_FRONT_RIGHT]),
                           (double)(game->derived.pureLateralForceN[WHEEL_REAR_LEFT] +
                                    game->derived.pureLateralForceN[WHEEL_REAR_RIGHT])),
                dim);
            hud_line(
                14, &y,
                TextFormat("limited Fx F/R %+.0f/%+.0f  limited Fy F/R %+.0f/%+.0f N",
                           (double)(game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                                    game->vehicle.wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN),
                           (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                                    game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN),
                           (double)game->derived.frontLateralForceN,
                           (double)game->derived.rearLateralForceN),
                dim);
            hud_line(14, &y,
                     TextFormat("usage FL/FR/RL/RR %.2f %.2f %.2f %.2f  lock %d%d%d%d",
                                (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                                (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage,
                                (double)game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                                (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage,
                                game->vehicle.wheels[WHEEL_FRONT_LEFT].locked,
                                game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked,
                                game->vehicle.wheels[WHEEL_REAR_LEFT].locked,
                                game->vehicle.wheels[WHEEL_REAR_RIGHT].locked),
                     dim);
            hud_line(14, &y,
                     TextFormat("brake F/R %.0f/%.0f Nm  handbrake R %.0f Nm",
                                (double)(game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                                         game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT]),
                                (double)(game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                                         game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT]),
                                (double)(game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
                                         game->derived.handbrakeTorqueNm[WHEEL_REAR_RIGHT])),
                     dim);
            hud_line(14, &y,
                     TextFormat("ax prev/filt/solved %+.2f/%+.2f/%+.2f m/s^2  "
                                "drag %.0f N  rolling %.0f N",
                                (double)game->derived.previousLongAccelMps2,
                                (double)game->derived.filteredLongAccelMps2,
                                (double)game->derived.solvedLongAccelMps2,
                                (double)game->derived.aeroDragMagnitudeN,
                                (double)game->derived.rollingResistanceMagnitudeN),
                     dim);
            draw_tire_curve_panel(game);
        }
    }
}

void render_hud_draw_state_overlay(const Game *game)
{
    switch (game->state) {
        case STATE_MENU: draw_overlay_menu(game); break;
        case STATE_PAUSED: draw_overlay_paused(game); break;
        case STATE_RESULTS: draw_overlay_results(game); break;
        default: break;
    }
}

#endif /* !CIRCUIT_HEADLESS */
