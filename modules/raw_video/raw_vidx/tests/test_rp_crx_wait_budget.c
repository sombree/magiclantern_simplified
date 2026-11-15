/* Host-side regression checks for bounded CRX completion waits.
 * gcc -std=c99 -Wall -Wextra -Werror -pedantic tests/test_rp_crx_wait_budget.c
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../rp_crx_wait_budget.h"

static void check(uint32_t elapsed, uint32_t expected)
{
    assert(rp_crx_wait_budget_ms(elapsed,1000000u,10u)==expected);
}

int main(void)
{
    check(0u,10u);
    check(1u,10u);
    check(988000u,10u);
    check(990000u,10u);
    check(990001u,10u);
    check(999000u,1u);
    check(999001u,1u);
    check(999999u,1u);
    check(1000000u,0u);
    check(1000001u,0u);
    assert(rp_crx_wait_budget_ms(123u,1000u,0u)==0u);
    assert(rp_crx_wait_budget_ms(10u,15u,1u)==1u);
    assert(rp_crx_wait_budget_ms(15u,15u,1u)==0u);
    assert(rp_crx_wait_budget_ms(0u,1u,10u)==1u);
    assert(rp_crx_wait_budget_ms(UINT32_MAX,UINT32_MAX,10u)==0u);
    puts("CRX event wait budget: PASS 15/15");
    return 0;
}
