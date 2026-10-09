#include <stdint.h>
#include <stdio.h>
#include "comms/fmt.h"

bool fmt_parse_float(const char *s, float *out)
{
    float sign = 1.0f, val = 0.0f, scale = 1.0f;
    bool digits = false;
    int exp = 0, esign = 1;

    if (*s == '+' || *s == '-')
        sign = (*s++ == '-') ? -1.0f : 1.0f;
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s++ - '0');
        digits = true;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            scale *= 0.1f;
            val += (float)(*s++ - '0') * scale;
            digits = true;
        }
    }
    if (!digits)
        return false;
    if (*s == 'e' || *s == 'E') {
        s++;
        if (*s == '+' || *s == '-')
            esign = (*s++ == '-') ? -1 : 1;
        if (!(*s >= '0' && *s <= '9'))
            return false;
        while (*s >= '0' && *s <= '9')
            exp = exp * 10 + (*s++ - '0');
        if (exp > 38)
            return false;
    }
    if (*s != '\0')
        return false;
    while (exp-- > 0)
        val = esign > 0 ? val * 10.0f : val / 10.0f;
    *out = sign * val;
    return true;
}

size_t fmt_float(char *buf, size_t len, float v, int decimals)
{
    uint32_t mul = 1;
    for (int i = 0; i < decimals; i++)
        mul *= 10;

    const char *sign = "";
    if (v < 0.0f) {
        sign = "-";
        v = -v;
    }
    if (v > 2.0e9f)
        return (size_t)snprintf(buf, len, "%sinf", sign);

    uint32_t ip = (uint32_t)v;
    uint32_t fp = (uint32_t)((v - (float)ip) * (float)mul + 0.5f);
    if (fp >= mul) {
        ip++;
        fp -= mul;
    }
    while (decimals > 0 && fp % 10 == 0) {
        fp /= 10;
        decimals--;
    }
    if (decimals == 0)
        return (size_t)snprintf(buf, len, "%s%lu", sign, (unsigned long)ip);
    return (size_t)snprintf(buf, len, "%s%lu.%0*lu", sign, (unsigned long)ip, decimals,
                            (unsigned long)fp);
}
