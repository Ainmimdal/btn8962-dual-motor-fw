#ifndef FMT_H
#define FMT_H

#include <stdbool.h>
#include <stddef.h>

/* Small float helpers so the build doesn't need printf/scanf float support. */
bool fmt_parse_float(const char *s, float *out);
/* Writes v with up to `decimals` fractional digits (trailing zeros trimmed). */
size_t fmt_float(char *buf, size_t len, float v, int decimals);

#endif
