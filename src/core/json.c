/*
 * json.c — strict deterministic JSON reader and canonical content hash. See json.h.
 */
#include "core/json.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------------------------------------- pools
 *
 * The document holds two bump pools plus a per-container growth model:
 *
 *   values    — one JsonValue per value in the document (a value is at least one character),
 *                bounded above by the input length.
 *   stringBuf — decoded string bytes; only ever shrinks relative to the source (escapes collapse
 *                to one byte), so it is allocated at input length.
 *
 * Object and array child pointers are NOT shared: each container owns its own realloc'd
 * keys/vals (objects) or items (arrays) array. A shared, monotonic child pool cannot keep a
 * parent's slice contiguous once a nested container appends its own children inside it, so the
 * per-container ownership is what makes nested structures correct. Each container's arrays are
 * freed by walking the value pool in json_document_free().
 */

#define JSON_MAX_DEPTH 200

struct JsonValue {
    JsonType type;
    double number;
    const char *string; /* STRING: decoded, NUL-terminated */
    /* ARRAY and OBJECT share the child storage model: pointers into one pool. */
    const JsonValue **items; /* ARRAY: itemCount entries */
    const char **keys;       /* OBJECT: memberCount entries */
    const JsonValue **vals;  /* OBJECT: memberCount entries */
    int itemCount;
    int memberCount;
};

struct JsonDocument {
    JsonValue *values;
    int valueCount;
    int valueCap;
    char *stringBuf;
    int stringLen;
    int stringCap;
    JsonValue root; /* by value: a NULL document still has a zeroed root */
};

/* -------------------------------------------------------------------------------------------- parser */

typedef struct {
    const char *text;
    size_t length;
    size_t pos;
    JsonDocument *doc;
    char *error;
    size_t errorCap;
    bool failed;
} Parser;

static void fail_at(Parser *p, size_t offset, const char *reason)
{
    if (p->failed) return;
    p->failed = true;
    if (p->error == NULL || p->errorCap == 0) return;
    int line = 1, col = 1;
    for (size_t i = 0; i < offset && i < p->length; i++) {
        if (p->text[i] == '\n') {
            line++;
            col = 1;
        } else {
            col++;
        }
    }
    snprintf(p->error, p->errorCap, "line %d, column %d: %s", line, col, reason);
}

static void fail_here(Parser *p, const char *reason)
{
    fail_at(p, p->pos, reason);
}

static JsonValue *alloc_value(Parser *p)
{
    if (p->doc->valueCount >= p->doc->valueCap) {
        fail_here(p, "document is too large for the value pool");
        return NULL;
    }
    JsonValue *v = &p->doc->values[p->doc->valueCount++];
    memset(v, 0, sizeof(*v));
    v->type = JSON_NULL;
    return v;
}

static void skip_ws(Parser *p)
{
    while (p->pos < p->length) {
        const char c = p->text[p->pos];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            p->pos++;
        } else {
            break;
        }
    }
}

static JsonValue *parse_value(Parser *p, int depth);

/* Decode a \uXXXX escape into a code point. Returns -1 on malformed input. */
static int decode_hex4(const char *s)
{
    int cp = 0;
    for (int i = 0; i < 4; i++) {
        const char c = s[i];
        int digit;
        if (c >= '0' && c <= '9') {
            digit = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            digit = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            digit = c - 'A' + 10;
        } else {
            return -1;
        }
        cp = (cp << 4) | digit;
    }
    return cp;
}

/* Encode one code point as UTF-8 into buf (up to 4 bytes + NUL). Returns bytes written or -1. */
static int encode_utf8(int cp, char *buf)
{
    if (cp < 0 || cp > 0x10FFFF) return -1;
    if (cp < 0x80) {
        buf[0] = (char)cp;
        return 1;
    }
    if (cp < 0x800) {
        buf[0] = (char)(0xC0 | (cp >> 6));
        buf[1] = (char)(0x80 | (cp & 0x3F));
        return 2;
    }
    if (cp < 0x10000) {
        buf[0] = (char)(0xE0 | (cp >> 12));
        buf[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
        buf[2] = (char)(0x80 | (cp & 0x3F));
        return 3;
    }
    buf[0] = (char)(0xF0 | (cp >> 18));
    buf[1] = (char)(0x80 | ((cp >> 12) & 0x3F));
    buf[2] = (char)(0x80 | ((cp >> 6) & 0x3F));
    buf[3] = (char)(0x80 | (cp & 0x3F));
    return 4;
}

/* Parse a quoted string starting at p->pos (which points at the opening quote). Appends the
 * decoded bytes to the document's string buffer and returns a pointer into it, or NULL. */
static const char *parse_string(Parser *p)
{
    if (p->pos >= p->length || p->text[p->pos] != '"') {
        fail_here(p, "expected a string");
        return NULL;
    }
    p->pos++; /* consume opening quote */
    const int startLen = p->doc->stringLen;
    char *buf = p->doc->stringBuf;
    int cap = p->doc->stringCap;

    while (p->pos < p->length) {
        const char c = p->text[p->pos];
        if (c == '"') {
            p->pos++; /* consume closing quote */
            if (p->doc->stringLen >= cap) {
                fail_here(p, "string buffer overflow");
                return NULL;
            }
            buf[p->doc->stringLen++] = '\0';
            return buf + startLen;
        }
        if ((unsigned char)c < 0x20) {
            fail_here(p, "unescaped control character in string");
            return NULL;
        }
        if (c == '\\') {
            p->pos++;
            if (p->pos >= p->length) {
                fail_here(p, "unterminated escape sequence");
                return NULL;
            }
            const char esc = p->text[p->pos++];
            char out;
            switch (esc) {
                case '"': out = '"'; break;
                case '\\': out = '\\'; break;
                case '/': out = '/'; break;
                case 'b': out = '\b'; break;
                case 'f': out = '\f'; break;
                case 'n': out = '\n'; break;
                case 'r': out = '\r'; break;
                case 't': out = '\t'; break;
                case 'u': {
                    if (p->pos + 4 > p->length) {
                        fail_here(p, "incomplete \\u escape");
                        return NULL;
                    }
                    int cp = decode_hex4(p->text + p->pos);
                    if (cp < 0) {
                        fail_here(p, "malformed \\u escape");
                        return NULL;
                    }
                    p->pos += 4;
                    /* Surrogate pair: a high surrogate must be followed by a low surrogate. */
                    if (cp >= 0xD800 && cp <= 0xDBFF) {
                        if (p->pos + 6 > p->length || p->text[p->pos] != '\\' ||
                            p->text[p->pos + 1] != 'u') {
                            fail_here(p, "lone high surrogate");
                            return NULL;
                        }
                        const int lo = decode_hex4(p->text + p->pos + 2);
                        if (lo < 0xD800 || lo > 0xDFFF) {
                            fail_here(p, "invalid low surrogate");
                            return NULL;
                        }
                        p->pos += 6;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
                        fail_here(p, "lone low surrogate");
                        return NULL;
                    }
                    if (cp == 0) {
                        fail_here(p, "NUL character is not permitted in string");
                        return NULL;
                    }
                    char ub[4];
                    const int n = encode_utf8(cp, ub);
                    if (n < 0) {
                        fail_here(p, "code point out of range");
                        return NULL;
                    }
                    for (int i = 0; i < n; i++) {
                        if (p->doc->stringLen >= cap) {
                            fail_here(p, "string buffer overflow");
                            return NULL;
                        }
                        buf[p->doc->stringLen++] = ub[i];
                    }
                    continue;
                }
                default: fail_here(p, "invalid escape sequence"); return NULL;
            }
            if (p->doc->stringLen >= cap) {
                fail_here(p, "string buffer overflow");
                return NULL;
            }
            buf[p->doc->stringLen++] = out;
            continue;
        }
        /* A plain byte, including the non-ASCII continuation bytes of any UTF-8 sequence: the
         * source is assumed to be valid UTF-8 and passed through verbatim. */
        if (p->doc->stringLen >= cap) {
            fail_here(p, "string buffer overflow");
            return NULL;
        }
        buf[p->doc->stringLen++] = c;
        p->pos++;
    }
    fail_here(p, "unterminated string");
    return NULL;
}

static JsonValue *parse_number(Parser *p)
{
    const size_t start = p->pos;
    const char *s = p->text + p->pos;

    /* Grammar check first so strtod cannot accept "NaN", "Infinity", or hex. */
    size_t i = 0;
    if (s[i] == '-') i++;
    if (i >= p->length - start || s[i] < '0' || s[i] > '9') {
        fail_here(p, "invalid number");
        return NULL;
    }
    /* Leading zeros: "0" alone is fine, "0123" is not JSON. */
    if (s[i] == '0') {
        i++;
    } else {
        while (i < p->length - start && s[i] >= '0' && s[i] <= '9') i++;
    }
    if (i < p->length - start && s[i] == '.') {
        i++;
        if (i >= p->length - start || s[i] < '0' || s[i] > '9') {
            fail_here(p, "invalid fraction in number");
            return NULL;
        }
        while (i < p->length - start && s[i] >= '0' && s[i] <= '9') i++;
    }
    if (i < p->length - start && (s[i] == 'e' || s[i] == 'E')) {
        i++;
        if (i < p->length - start && (s[i] == '+' || s[i] == '-')) i++;
        if (i >= p->length - start || s[i] < '0' || s[i] > '9') {
            fail_here(p, "invalid exponent in number");
            return NULL;
        }
        while (i < p->length - start && s[i] >= '0' && s[i] <= '9') i++;
    }

    /* Parse the validated token using a NUL-terminated stack buffer so strtod cannot read
     * past the end of length-delimited input slices. */
    char numBuf[128];
    char *numStr = numBuf;
    if (i >= sizeof(numBuf)) {
        numStr = (char *)malloc(i + 1);
        if (numStr == NULL) {
            fail_here(p, "out of memory");
            return NULL;
        }
    }
    memcpy(numStr, s, i);
    numStr[i] = '\0';
    char *endptr = NULL;
    const double value = strtod(numStr, &endptr);
    (void)endptr;
    if (numStr != numBuf) free(numStr);
    p->pos = start + i;

    JsonValue *v = alloc_value(p);
    if (v == NULL) return NULL;
    v->type = JSON_NUMBER;
    v->number = value;
    if (!isfinite(value)) {
        fail_at(p, start, "number is not finite");
        return NULL;
    }
    return v;
}

static JsonValue *parse_array(Parser *p, int depth)
{
    p->pos++; /* consume '[' */
    JsonValue *v = alloc_value(p);
    if (v == NULL) return NULL;
    v->type = JSON_ARRAY;
    int cap = 0; /* capacity of v->items, tracked locally as it grows */
    skip_ws(p);
    if (p->pos < p->length && p->text[p->pos] == ']') {
        p->pos++;
        return v; /* empty array: items == NULL, itemCount == 0 */
    }
    for (;;) {
        const JsonValue *item = parse_value(p, depth + 1);
        if (item == NULL) return NULL; /* v->items, if any, is freed with the document */
        if (v->itemCount >= cap) {
            const int ncap = (cap == 0) ? 4 : cap * 2;
            const JsonValue **grown =
                (const JsonValue **)realloc(v->items, (size_t)ncap * sizeof(*grown));
            if (grown == NULL) {
                fail_here(p, "out of memory while parsing array");
                return NULL;
            }
            v->items = grown;
            cap = ncap;
        }
        v->items[v->itemCount++] = item;
        skip_ws(p);
        if (p->pos >= p->length) {
            fail_here(p, "unterminated array");
            return NULL;
        }
        const char c = p->text[p->pos];
        if (c == ']') {
            p->pos++;
            break;
        }
        if (c != ',') {
            fail_here(p, "expected ',' or ']' in array");
            return NULL;
        }
        p->pos++;
        skip_ws(p);
    }
    return v;
}

static JsonValue *parse_object(Parser *p, int depth)
{
    p->pos++; /* consume '{' */
    JsonValue *v = alloc_value(p);
    if (v == NULL) return NULL;
    v->type = JSON_OBJECT;
    int cap = 0; /* shared capacity of v->keys and v->vals as they grow in parallel */
    skip_ws(p);
    if (p->pos < p->length && p->text[p->pos] == '}') {
        p->pos++;
        return v; /* empty object */
    }
    for (;;) {
        skip_ws(p);
        const size_t keyPos = p->pos;
        const char *key = parse_string(p);
        if (key == NULL) return NULL; /* v->keys/vals, if any, freed with the document */
        skip_ws(p);
        if (p->pos >= p->length || p->text[p->pos] != ':') {
            fail_here(p, "expected ':' after object key");
            return NULL;
        }
        p->pos++;
        skip_ws(p);
        const JsonValue *val = parse_value(p, depth + 1);
        if (val == NULL) return NULL;

        /* Duplicate key check before inserting, using exact key position for diagnostics. */
        for (int m = 0; m < v->memberCount; m++) {
            if (strcmp(v->keys[m], key) == 0) {
                fail_at(p, keyPos, "duplicate object key");
                return NULL;
            }
        }

        /* Append the (key, value) pair to this object's own arrays. Allocate into temporary
         * pointers before modifying v->keys / v->vals to prevent dangling pointers on OOM. */
        if (v->memberCount >= cap) {
            const int ncap = (cap == 0) ? 4 : cap * 2;
            const char **kg =
                (const char **)realloc((void *)v->keys, (size_t)ncap * sizeof(*kg));
            if (kg == NULL) {
                fail_here(p, "out of memory while parsing object");
                return NULL;
            }
            v->keys = kg;
            const JsonValue **vg =
                (const JsonValue **)realloc((void *)v->vals, (size_t)ncap * sizeof(*vg));
            if (vg == NULL) {
                fail_here(p, "out of memory while parsing object");
                return NULL;
            }
            v->vals = vg;
            cap = ncap;
        }
        v->keys[v->memberCount] = key;
        v->vals[v->memberCount] = val;
        v->memberCount++;

        skip_ws(p);
        if (p->pos >= p->length) {
            fail_here(p, "unterminated object");
            return NULL;
        }
        const char c = p->text[p->pos];
        if (c == '}') {
            p->pos++;
            break;
        }
        if (c != ',') {
            fail_here(p, "expected ',' or '}' in object");
            return NULL;
        }
        p->pos++;
    }
    return v;
}
static JsonValue *parse_literal(Parser *p, const char *text, JsonType type, double number)
{
    const size_t len = strlen(text);
    if (p->pos + len > p->length || memcmp(p->text + p->pos, text, len) != 0) {
        fail_here(p, "invalid literal");
        return NULL;
    }
    p->pos += len;
    JsonValue *v = alloc_value(p);
    if (v == NULL) return NULL;
    v->type = type;
    v->number = number;
    return v;
}

static JsonValue *parse_value(Parser *p, int depth)
{
    if (depth > JSON_MAX_DEPTH) {
        fail_here(p, "nesting exceeds the depth limit");
        return NULL;
    }
    skip_ws(p);
    if (p->pos >= p->length) {
        fail_here(p, "unexpected end of input");
        return NULL;
    }
    const char c = p->text[p->pos];
    switch (c) {
        case '{': return parse_object(p, depth);
        case '[': return parse_array(p, depth);
        case '"': {
            JsonValue *v = alloc_value(p);
            if (v == NULL) return NULL;
            const char *s = parse_string(p);
            if (s == NULL) return NULL;
            v->type = JSON_STRING;
            v->string = s;
            return v;
        }
        case 't': return parse_literal(p, "true", JSON_BOOL, 1.0);
        case 'f': return parse_literal(p, "false", JSON_BOOL, 0.0);
        case 'n': return parse_literal(p, "null", JSON_NULL, 0.0);
        default:
            if (c == '-' || (c >= '0' && c <= '9')) return parse_number(p);
            fail_here(p, "unexpected character");
            return NULL;
    }
}

JsonDocument *json_parse(const char *text, size_t length, char *error, size_t errorCap)
{
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (text == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "empty input");
        return NULL;
    }
    /* A zero length is a convenience that means "strlen(text)". An empty string still parses to
     * length zero and fails below, so this does not weaken the empty-input rejection. */
    if (length == 0) length = strlen(text);
    if (length == 0) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "empty input");
        return NULL;
    }

    JsonDocument *doc = (JsonDocument *)calloc(1, sizeof(JsonDocument));
    if (doc == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "out of memory");
        return NULL;
    }

    /* Pool capacities are bounded by the input: a value needs at least one character and a decoded
     * string never exceeds the source length. Object and array child arrays grow per container, so
     * no child/key pool is pre-allocated here. */
    doc->valueCap = (int)length + 1;
    doc->stringCap = (int)length + 1;
    doc->values = (JsonValue *)calloc((size_t)doc->valueCap, sizeof(JsonValue));
    doc->stringBuf = (char *)calloc((size_t)doc->stringCap, 1);
    if (doc->values == NULL || doc->stringBuf == NULL) {
        json_document_free(doc);
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "out of memory");
        return NULL;
    }

    Parser p;
    memset(&p, 0, sizeof(p));
    p.text = text;
    p.length = length;
    p.doc = doc;
    p.error = error;
    p.errorCap = errorCap;

    const JsonValue *root = parse_value(&p, 0);
    if (p.failed || root == NULL) {
        json_document_free(doc);
        return NULL;
    }
    skip_ws(&p);
    if (p.pos != p.length) {
        fail_here(&p, "trailing characters after top-level value");
        json_document_free(doc);
        return NULL;
    }
    doc->root = *root;
    return doc;
}

void json_document_free(JsonDocument *doc)
{
    if (doc == NULL) return;
    /* Each container owns its keys/vals/items arrays; free them as we walk the value pool. */
    for (int i = 0; i < doc->valueCount; i++) {
        JsonValue *v = &doc->values[i];
        free(v->keys);
        free(v->vals);
        free(v->items);
    }
    free(doc->values);
    free(doc->stringBuf);
    free(doc);
}

const JsonValue *json_document_root(const JsonDocument *doc)
{
    return (doc != NULL) ? &doc->root : NULL;
}

/* --------------------------------------------------------------------------------------------- access */

JsonType json_type(const JsonValue *value)
{
    return (value == NULL) ? JSON_NULL : value->type;
}

bool json_is_null(const JsonValue *value)
{
    return json_type(value) == JSON_NULL;
}
bool json_is_bool(const JsonValue *value)
{
    return json_type(value) == JSON_BOOL;
}
bool json_is_number(const JsonValue *value)
{
    return json_type(value) == JSON_NUMBER;
}
bool json_is_string(const JsonValue *value)
{
    return json_type(value) == JSON_STRING;
}
bool json_is_array(const JsonValue *value)
{
    return json_type(value) == JSON_ARRAY;
}
bool json_is_object(const JsonValue *value)
{
    return json_type(value) == JSON_OBJECT;
}

double json_as_number(const JsonValue *value)
{
    return (value != NULL && value->type == JSON_NUMBER) ? value->number : 0.0;
}
bool json_as_bool(const JsonValue *value)
{
    return (value != NULL && value->type == JSON_BOOL) ? value->number != 0.0 : false;
}
const char *json_as_string(const JsonValue *value)
{
    return (value != NULL && value->type == JSON_STRING) ? value->string : NULL;
}

int json_array_count(const JsonValue *value)
{
    return (value != NULL && value->type == JSON_ARRAY) ? value->itemCount : 0;
}
const JsonValue *json_array_at(const JsonValue *value, int index)
{
    if (value == NULL || value->type != JSON_ARRAY || index < 0 || index >= value->itemCount)
        return NULL;
    return value->items[index];
}

int json_object_count(const JsonValue *value)
{
    return (value != NULL && value->type == JSON_OBJECT) ? value->memberCount : 0;
}
const char *json_object_key_at(const JsonValue *value, int index)
{
    if (value == NULL || value->type != JSON_OBJECT || index < 0 || index >= value->memberCount)
        return NULL;
    return value->keys[index];
}
const JsonValue *json_object_value_at(const JsonValue *value, int index)
{
    if (value == NULL || value->type != JSON_OBJECT || index < 0 || index >= value->memberCount)
        return NULL;
    return value->vals[index];
}
bool json_object_has(const JsonValue *value, const char *key)
{
    return json_object_get(value, key) != NULL;
}
const JsonValue *json_object_get(const JsonValue *value, const char *key)
{
    if (value == NULL || value->type != JSON_OBJECT || key == NULL) return NULL;
    for (int i = 0; i < value->memberCount; i++) {
        if (strcmp(value->keys[i], key) == 0) return value->vals[i];
    }
    return NULL;
}

/* ---------------------------------------------------------------------------------------- canonical hash */

#define FNV1A_OFFSET_BASIS 2166136261u
#define FNV1A_PRIME 16777619u

static uint32_t hash_mix_u32(uint32_t h, uint32_t value)
{
    const unsigned char *bytes = (const unsigned char *)&value;
    for (size_t i = 0; i < sizeof(value); i++) {
        h ^= (uint32_t)bytes[i];
        h *= FNV1A_PRIME;
    }
    return h;
}

static uint32_t hash_mix_bytes(uint32_t h, const char *bytes, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        h ^= (uint32_t)(unsigned char)bytes[i];
        h *= FNV1A_PRIME;
    }
    return h;
}

static uint32_t hash_string(uint32_t h, const char *tag, const char *s)
{
    h = hash_mix_bytes(h, tag, 1);
    if (s != NULL) {
        const size_t len = strlen(s);
        h = hash_mix_u32(h, (uint32_t)len);
        h = hash_mix_bytes(h, s, len);
    }
    return h;
}

static uint32_t hash_value(const JsonValue *v, uint32_t h);

static uint32_t hash_value(const JsonValue *v, uint32_t h)
{
    if (v == NULL) return hash_mix_bytes(h, "N", 1);
    switch (v->type) {
        case JSON_NULL: return hash_mix_bytes(h, "N", 1);
        case JSON_BOOL: {
            h = hash_mix_bytes(h, "B", 1);
            return hash_mix_u32(h, v->number != 0.0 ? 1u : 0u);
        }
        case JSON_NUMBER: {
            h = hash_mix_bytes(h, "n", 1);
            double d = v->number;
            uint64_t bits;
            memcpy(&bits, &d, sizeof(bits));
            /* Normalize -0.0 to +0.0 so the two zero spellings hash identically: clear the sign
             * bit only when the magnitude is zero. */
            if ((bits & 0x7FFFFFFFFFFFFFFFULL) == 0u) bits = 0u;
            h = hash_mix_u32(h, (uint32_t)(bits & 0xFFFFFFFFu));
            return hash_mix_u32(h, (uint32_t)(bits >> 32));
        }
        case JSON_STRING: return hash_string(h, "s", v->string);
        case JSON_ARRAY: {
            h = hash_mix_bytes(h, "a", 1);
            h = hash_mix_u32(h, (uint32_t)v->itemCount);
            for (int i = 0; i < v->itemCount; i++) h = hash_value(v->items[i], h);
            return h;
        }
        case JSON_OBJECT: {
            h = hash_mix_bytes(h, "o", 1);
            h = hash_mix_u32(h, (uint32_t)v->memberCount);
            /* Members are hashed in sorted-key order so reordering does not change the hash.
             * Selection pass uses no heap memory so hash calculation is always deterministic. */
            const int n = v->memberCount;
            if (n > 0) {
                const char *prevKey = NULL;
                for (int pass = 0; pass < n; pass++) {
                    int best = -1;
                    for (int i = 0; i < n; i++) {
                        if (prevKey != NULL && strcmp(v->keys[i], prevKey) <= 0) continue;
                        if (best < 0 || strcmp(v->keys[i], v->keys[best]) < 0) {
                            best = i;
                        }
                    }
                    if (best >= 0) {
                        prevKey = v->keys[best];
                        h = hash_string(h, "k", v->keys[best]);
                        h = hash_value(v->vals[best], h);
                    }
                }
            }
            return h;
        }
    }
    return h;
}

uint32_t json_canonical_hash(const JsonValue *root)
{
    if (root == NULL) return 0u;
    return hash_value(root, FNV1A_OFFSET_BASIS);
}
