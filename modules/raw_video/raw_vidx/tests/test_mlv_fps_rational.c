/* Host-side test: gcc -std=c99 -Wall -Wextra -Werror -pedantic
 * tests/test_mlv_fps_rational.c -o /tmp/test_mlv_fps && /tmp/test_mlv_fps */
#include <stdint.h>
#include <stdio.h>

#include "../mlv_fps_rational.h"

struct fps_case {
    int input;
    uint32_t expected_nom;
    uint32_t expected_denom;
};

int main(void)
{
    static const struct fps_case cases[] = {
        {0, 1, 1000}, {-1, 1, 1000},
        {23976, 24000, 1001}, {29970, 30000, 1001},
        {59940, 60000, 1001}, {119880, 120000, 1001},
        {47952, 48000, 1001},
        {23975, 23975, 1000}, {23977, 23977, 1000},
        {29969, 29969, 1000}, {29971, 29971, 1000},
        {24000, 24000, 1000}, {25000, 25000, 1000},
        {30000, 30000, 1000}, {50000, 50000, 1000},
        {60000, 60000, 1000}, {1000, 1000, 1000},
        {1, 1, 1000}, {12345, 12345, 1000},
        {2147483647, 2147483647u, 1000}
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        uint32_t nom = 0u, denom = 0u;
        mlv_fps_to_rational(cases[i].input, &nom, &denom);
        if (nom != cases[i].expected_nom || denom != cases[i].expected_denom)
        {
            fprintf(stderr, "FAIL input=%d got %u/%u wanted %u/%u\n",
                    cases[i].input, nom, denom,
                    cases[i].expected_nom, cases[i].expected_denom);
            return 1;
        }
    }
    printf("MLV FPS rational: %u/%u tests PASS\n",
           (unsigned)(sizeof(cases) / sizeof(cases[0])),
           (unsigned)(sizeof(cases) / sizeof(cases[0])));
    return 0;
}
