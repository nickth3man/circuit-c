/*
 * json.h — a strict, deterministic JSON reader plus a canonical content hash.
 *
 * WHY JSON. The vehicle and track content formats (issues #29 and #34) need a human-reviewable,
 * versioned, cross-platform representation. JSON is the boring, widely-understood choice, and a
 * single strict reader serves both loaders instead of two bespoke text formats.

 * WHY STRICT. A content manifest that silently accepts a typo or a trailing comma turns a data
 * error into a wrong car or a wrong track. The reader rejects every deviation from RFC 8259 that
 * could make two byte-different files load as the same value (duplicate keys, trailing tokens,
 * comments, NaN/Infinity), and reports each as a one-line `line N, column M: reason` error so a
 * loader can surface the field that owns it.
 *
 * DETERMINISM. Numbers parse through strtod into a double. The supported toolchains (mingw-w64
 * UCRT64 and glibc/clang libc) provide IEEE-754 correctly-rounded strtod, so the same numeric
 * text yields the same double on every supported platform; manifests and tracks therefore hash
 * identically regardless of where they were loaded. The canonical hash below documents that
 * assumption rather than hiding it.
 *
 * This translation unit calls no raylib function and holds no module-static state, so it links
 * into the hot-reloadable game module and the headless test executable without a reload hazard.
 */
#ifndef CIRCUIT_JSON_H
#define CIRCUIT_JSON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    JSON_NULL = 0,
    JSON_BOOL,
    JSON_NUMBER,
    JSON_STRING,
    JSON_ARRAY,
    JSON_OBJECT
} JsonType;

typedef struct JsonValue JsonValue;
typedef struct JsonDocument JsonDocument;

/* Parse `length` bytes of UTF-8 JSON text (pass 0 for `length` to use strlen(text)). On success
 * returns a document whose root is reached through json_document_root(); on failure returns NULL and,
 * when error/errorCap are non-NULL, writes a `line N, column M: reason` message. The document owns
 * all of its memory; free it with json_document_free(). NULL text or empty input is a clean parse
 * failure, not a crash. */
JsonDocument *json_parse(const char *text, size_t length, char *error, size_t errorCap);

void json_document_free(JsonDocument *doc);

/* The top-level value, or NULL if the document is NULL. */
const JsonValue *json_document_root(const JsonDocument *doc);

JsonType json_type(const JsonValue *value);
bool json_is_null(const JsonValue *value);
bool json_is_bool(const JsonValue *value);
bool json_is_number(const JsonValue *value);
bool json_is_string(const JsonValue *value);
bool json_is_array(const JsonValue *value);
bool json_is_object(const JsonValue *value);

/* Type-coercing accessors. Each returns the zero value of its type when `value` is the wrong
 * type (0.0, false, NULL, -1), so a caller that has already checked the type does not have to
 * special-case a NULL argument as well. */
double json_as_number(const JsonValue *value);
bool json_as_bool(const JsonValue *value);
const char *json_as_string(const JsonValue *value);

/* Arrays. json_array_at returns NULL for an out-of-range index, so bounds checks compose. */
int json_array_count(const JsonValue *value);
const JsonValue *json_array_at(const JsonValue *value, int index);

/* Objects, in source order. json_object_get returns NULL when the key is absent, which is how a
 * loader distinguishes a missing optional field from a present null one. */
int json_object_count(const JsonValue *value);
const char *json_object_key_at(const JsonValue *value, int index);
const JsonValue *json_object_value_at(const JsonValue *value, int index);
bool json_object_has(const JsonValue *value, const char *key);
const JsonValue *json_object_get(const JsonValue *value, const char *key);

/*
 * FNV-1a 32-bit over the value tree in a canonical form that does not depend on the source
 * formatting: object members are hashed in sorted-key order, numbers by their IEEE-754 double
 * bits (with -0.0 normalized to +0.0), and strings by their decoded UTF-8 bytes. Two documents
 * that mean the same thing therefore hash the same, which is what content-compatibility and
 * replay/save checks rely on. Returns 0 for a NULL root.
 */
uint32_t json_canonical_hash(const JsonValue *root);

#endif /* CIRCUIT_JSON_H */
