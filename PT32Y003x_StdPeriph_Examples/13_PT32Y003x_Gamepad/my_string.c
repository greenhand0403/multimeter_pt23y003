#include "my_string.h"
int my_strlen(const char *s)
{
    int n = 0;
    while (*s++) n++;
    return n;
}
int my_strcmp(const char *a, const char *b)
{
    while (*a && (*a == *b)) {
        a++; b++;
    }
    return (unsigned char)*a - (unsigned char)*b;
}
const char* my_strstr(const char *hay, const char *needle)
{
    if (!*needle) return hay;

    for (; *hay; hay++) {
        if (*hay == *needle) {
            const char *h = hay, *n = needle;
            while (*h && *n && *h == *n) {
                h++; n++;
            }
            if (!*n) return hay;
        }
    }
    return 0;
}
void my_strncpy(char *dst, const char *src, int n)
{
    while (n-- > 0 && (*dst++ = *src++));
}
void my_memcpy(void *dst, const void *src, int n)
{
    unsigned char *d = (unsigned char*)dst;
    const unsigned char *s = (const unsigned char*)src;
    while (n--) *d++ = *s++;
}