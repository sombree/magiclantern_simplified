/* EOS RP.160 resident RPSS114 generation-isolated SSRaw lease broker.
 *
 * Broker state is resident in the RP.160 platform core.
 * Keep in the platform core, rather than a dynamically unloadable module,
 * because native Canon async callbacks may be delayed across a timeout.
 * Do not also enable a copy of this broker in src/debug.c.
 * No ROM hooks, MMU remapping, or general-purpose SRM allocation.
 */
#include "dryos.h"
#include "lens.h"
#include "shoot.h"
#include <stdint.h>

/* EOS RP.160 SSRaw-A broker dependencies (native job 0x2C).
 * Keep the verified native ABI and preflight gates. These are required by
 * raw_vidx through rp_ssraw_v110_* and are NOT a general-purpose SRM heap.
 * RPSS103 one-shot test, RPSS104/105 MMU mapping experiments and their menu
 * options have been removed from the core to reduce boot image footprint.
 */
#define RP_SS103_ALLOC_THUMB  0xE052E4DDu
#define RP_SS103_FREE_THUMB   0xE052E55Fu
#define RP_SS103_CTX_SLOT     0x00005158u
#define RP_SS103_OCC_WORD1    0x00038118u
#define RP_SS103_BANK_BYTES   0x077B0000u
#define RP_SS103_A_RAW        0xA0000000u
#define RP_SS103_B_RAW        0x60000000u
#define RP_SS103_ROM_A        0xE0A27B54u
#define RP_SS103_ROM_B        0xE0A27B5Cu
#define RP_SS103_FLAG_A       (1u << 24)
#define RP_SS103_FLAG_B       (1u << 25)
#define RP_SS103_A_BLOCKERS   ((1u << 13) | RP_SS103_FLAG_A)
#define RP_SS103_B_BLOCKERS   ((1u << 5) | (1u << 6) | (1u << 8) | \
                               (1u << 11) | (1u << 12) | (1u << 16) | \
                               (1u << 27) | RP_SS103_FLAG_B)

typedef void (*rp_ss103_alloc_api_t)(uint32_t, uint32_t,
    void (*)(void *, uintptr_t, uint32_t), void *);
typedef void (*rp_ss103_free_api_t)(const uint32_t *, uint32_t,
    void (*)(void *, uint32_t, uint32_t), void *);

/* Preserved so the RPSS114 first-acquire exclusion is unchanged. */
static volatile int rp_ss103_attempted=0;

/* Host tests may substitute a passive register reader; production always
 * reads the exact Canon RP.160 addresses. */
#ifndef RP_SS114_READ32
#define RP_SS114_READ32(addr) (*(volatile const uint32_t *)(uintptr_t)(addr))
#endif
static uint32_t rp_ss103_r32(uint32_t addr)
{
    return RP_SS114_READ32(addr);
}

static int rp_ss103_ctx_valid(uint32_t ctx)
{
    return (ctx & 3u)==0u && ctx >= 0x00010000u && ctx < 0x10000000u;
}

static int rp_ss103_rom_ok(void)
{
    return rp_ss103_r32(RP_SS103_ROM_A) == RP_SS103_A_RAW &&
           rp_ss103_r32(RP_SS103_ROM_A + 4u) == RP_SS103_BANK_BYTES &&
           rp_ss103_r32(RP_SS103_ROM_B) == RP_SS103_B_RAW &&
           rp_ss103_r32(RP_SS103_ROM_B + 4u) == RP_SS103_BANK_BYTES;
}

/* Same predicate as the old rp_ss103_preflight(0), without the obsolete
 * menu/reporting branch. Never allocates or frees Canon memory. */
static int rp_ss103_preflight(void)
{
    uint32_t ctx = rp_ss103_r32(RP_SS103_CTX_SLOT);
    uint32_t word = rp_ss103_r32(RP_SS103_OCC_WORD1);
    uint32_t mode = rp_ss103_ctx_valid(ctx) ? rp_ss103_r32(ctx+0x150u) : 0xFFFFFFFFu;
    int rom = rp_ss103_rom_ok();
    int busy = lens_info.job_state || RECORDING_H264;
    int a_open = !(word & RP_SS103_A_BLOCKERS);
    int b_blocked = !!(word & RP_SS103_B_BLOCKERS);
    return rom && rp_ss103_ctx_valid(ctx) && mode==0 && !busy && a_open && b_blocked;
}

/* RPSS114: generation-isolated, repeatable SSRaw-A lease broker (RP.160).
 * Resident CORE owns every Canon callback/cookie; raw_vidx still owns the
 * frame slot descriptors and must drain FIO before calling release().
 * Every native operation gets a never-reused cookie address this boot.
 * A callback timeout does not cancel a Canon job. No blind retry or free.
 * CONFIG_MEMORY_SRM_NOT_WORKING remains set: this is NOT a generic SRM heap.
 *
 * For compatibility the exported API retains its v110 symbol names.
 */
#define RP_SS114_MAX_SESSIONS 32u
#ifdef RP_SS110_HOST_TEST
#define RP_SS110_WAIT_MS 120u
#define RP_SS110_POLL_MS 1u
#else
#define RP_SS110_WAIT_MS 2000u
#define RP_SS110_POLL_MS 20u
#endif

#ifndef RP_SS110_NATIVE_ALLOC
#define RP_SS110_NATIVE_ALLOC(n, cb, cookie) \
    ((rp_ss103_alloc_api_t)RP_SS103_ALLOC_THUMB)(0u,(n),(cb),(cookie))
#endif
#ifndef RP_SS110_NATIVE_FREE
#define RP_SS110_NATIVE_FREE(addr, cb, cookie) \
    ((rp_ss103_free_api_t)RP_SS103_FREE_THUMB)((addr),1u,(cb),(cookie))
#endif

enum rp_ss110_state {
    RP_SS110_UNUSED=0, RP_SS110_START=1, RP_SS110_PENDING=2,
    RP_SS110_HELD=3, RP_SS110_RELEASE_REQUEST=4,
    RP_SS110_FREE_PENDING=5, RP_SS110_RELEASED=6,
    RP_SS110_FAILED=7, RP_SS110_QUARANTINE=8,
    RP_SS114_OFFERED=9, RP_SS114_PREPARE=10
};

struct rp_ss114_free_cookie {
    uint32_t generation, slot;
};
struct rp_ss114_round {
    volatile uint32_t alloc_seen, alloc_publish;
    volatile uint32_t count, error_code, addresses[2];
    volatile uint32_t free_seen[2], free_publish[2];
    volatile uint32_t free_r1[2], free_r2[2];
    uint32_t before_word1;
    struct rp_ss114_free_cookie free_cookie[2];
};
static struct rp_ss114_round rp_ss114_rounds[RP_SS114_MAX_SESSIONS];
static struct rp_ss110_core {
    volatile uint32_t state, cancelled, bad_callbacks, release_timeout_latch;
    volatile uint32_t active_generation, generations_used;
    volatile uint32_t task_started;
} rp_ss110;

/* Reject arbitrary callback cookie pointers without dereferencing them. The
 * session objects and their cookies never move or get reused. */
static struct rp_ss114_round *rp_ss114_find_alloc_cookie(void *cookie)
{
    for (uint32_t i=0; i<RP_SS114_MAX_SESSIONS; ++i)
        if (cookie==(void *)&rp_ss114_rounds[i])
            return &rp_ss114_rounds[i];
    return NULL;
}
static struct rp_ss114_round *rp_ss114_find_free_cookie(void *cookie,
                                                         uint32_t *slot)
{
    for (uint32_t i=0; i<RP_SS114_MAX_SESSIONS; ++i)
        for (uint32_t j=0; j<2u; ++j)
            if (cookie==(void *)&rp_ss114_rounds[i].free_cookie[j]) {
                struct rp_ss114_free_cookie *tag=&rp_ss114_rounds[i].free_cookie[j];
                if (tag->generation!=i || tag->slot!=j) return NULL;
                *slot=j;
                return &rp_ss114_rounds[i];
            }
    return NULL;
}

/* Canon resource callbacks: bounded stores, fences and atomics only. */
#ifndef RP_SS114_GRANT_WORD
#define RP_SS114_GRANT_WORD(raw, n) (((const uint32_t *)(uintptr_t)(raw))[n])
#endif
static void rp_ss114_acq_cb(void *cookie, uintptr_t raw, uint32_t count)
{
    struct rp_ss114_round *r=rp_ss114_find_alloc_cookie(cookie);
    if (!r) {
        __sync_add_and_fetch(&rp_ss110.bad_callbacks,1);
        return;
    }
    __sync_add_and_fetch(&r->alloc_seen,1);
    if (!__sync_bool_compare_and_swap(&r->alloc_publish,0u,1u)) {
        __sync_add_and_fetch(&rp_ss110.bad_callbacks,1);
        return;
    }
    r->count=count;
    if (count==0u) {
        r->error_code=(uint32_t)raw; /* scalar error, not a pointer */
    } else if (count>2u || raw<0x10000u || raw>=0x10000000u || (raw&3u)) {
        r->error_code=0xBAD00114u;
    } else {
        r->addresses[0]=RP_SS114_GRANT_WORD(raw,0u);
        if (count==2u) r->addresses[1]=RP_SS114_GRANT_WORD(raw,1u);
    }
    __sync_synchronize();
    r->alloc_publish=2u;
}

static void rp_ss114_free_cb(void *cookie, uint32_t r1, uint32_t r2)
{
    uint32_t slot=0;
    struct rp_ss114_round *r=rp_ss114_find_free_cookie(cookie,&slot);
    if (!r) {
        __sync_add_and_fetch(&rp_ss110.bad_callbacks,1);
        return;
    }
    __sync_add_and_fetch(&r->free_seen[slot],1);
    if (!__sync_bool_compare_and_swap(&r->free_publish[slot],0u,1u)) {
        __sync_add_and_fetch(&rp_ss110.bad_callbacks,1);
        return;
    }
    r->free_r1[slot]=r1;
    r->free_r2[slot]=r2;
    __sync_synchronize();
    r->free_publish[slot]=2u;
}

static int rp_ss114_known_bank(uint32_t address)
{
    return address==RP_SS103_A_RAW || address==RP_SS103_B_RAW;
}

/* Only the resident worker calls native free; no caller can cancel a
 * pending free or reclaim a buffer while a Canon callback remains pending. */
static int rp_ss114_release_core(struct rp_ss114_round *r, uint32_t generation)
{
    struct rp_ss110_core *c=&rp_ss110;
    c->state=RP_SS110_FREE_PENDING;
    __sync_synchronize();
    for (uint32_t i=0; i<r->count; ++i) {
        uint32_t addr=r->addresses[i];
        if (!rp_ss114_known_bank(addr) || i>=2u) {
            DryosDebugMsg(0,15,"RPSS114 UNSAFE_FREE gen=%d slot=%d addr=%x",
                          generation,i,addr);
            return -1;
        }
        r->free_r1[i]=r->free_r2[i]=0xFFFFFFFFu;
        __sync_synchronize();
        RP_SS110_NATIVE_FREE(&addr,rp_ss114_free_cb,&r->free_cookie[i]);
        while (r->free_publish[i]!=2u) {
            if (c->bad_callbacks) return -2;
            msleep(RP_SS110_POLL_MS);
        }
        __sync_synchronize();
        if (r->free_seen[i]!=1u || r->free_r1[i] || r->free_r2[i] ||
            c->bad_callbacks) {
            DryosDebugMsg(0,15,"RPSS114 FREE_BAD gen=%d slot=%d cb=%d r1=%x r2=%x",
                 generation,i,r->free_seen[i],r->free_r1[i],r->free_r2[i]);
            return -3;
        }
        DryosDebugMsg(0,15,"RPSS114 FREE_ACK gen=%d slot=%d addr=%x",
                      generation,i,addr);
    }
    uint32_t now=rp_ss103_r32(RP_SS103_OCC_WORD1);
    if (((now ^ r->before_word1) & (RP_SS103_FLAG_A | RP_SS103_FLAG_B)) ||
        (now & RP_SS103_FLAG_A) || c->bad_callbacks) {
        DryosDebugMsg(0,15,"RPSS114 RESTORE_BAD gen=%d before=%x now=%x",
                      generation,r->before_word1,now);
        return -4;
    }
    return 0;
}

/* This task never unloads. It serializes all native jobs, and a missing
 * callback parks it indefinitely rather than accepting another generation. */
static void rp_ss114_task(void *arg)
{
    (void)arg;
    struct rp_ss110_core *c=&rp_ss110;
    for (;;) {
        if (c->bad_callbacks && c->state!=RP_SS110_QUARANTINE)
            c->state=RP_SS110_QUARANTINE;
        if (c->state!=RP_SS110_START) {
            msleep(RP_SS110_POLL_MS);
            continue;
        }
        uint32_t generation=c->active_generation;
        if (generation>=RP_SS114_MAX_SESSIONS) {
            c->state=RP_SS110_QUARANTINE;
            continue;
        }
        struct rp_ss114_round *r=&rp_ss114_rounds[generation];
        /* If acquire timed out before the job was issued, do not issue it. */
        if (c->cancelled) {
            c->state=RP_SS110_FAILED;
            continue;
        }
        r->before_word1=rp_ss103_r32(RP_SS103_OCC_WORD1);
        c->state=RP_SS110_PENDING;
        __sync_synchronize();
        DryosDebugMsg(0,15,"RPSS114 ALLOC gen=%d slots=2 before=%x",
                      generation,r->before_word1);
        RP_SS110_NATIVE_ALLOC(2u,rp_ss114_acq_cb,r);
        while (r->alloc_publish!=2u) {
            if (c->bad_callbacks) c->state=RP_SS110_QUARANTINE;
            msleep(RP_SS110_POLL_MS);
        }
        __sync_synchronize();
        if (c->bad_callbacks || r->alloc_seen!=1u) {
            c->state=RP_SS110_QUARANTINE;
            continue;
        }
        if (r->count==0u) {
            DryosDebugMsg(0,15,"RPSS114 DENIED gen=%d code=%x",
                          generation,r->error_code);
            c->state=RP_SS110_FAILED;
            continue;
        }
        if (r->error_code || r->count>2u ||
            !rp_ss114_known_bank(r->addresses[0]) ||
            (r->count==2u && (!rp_ss114_known_bank(r->addresses[1]) ||
                              r->addresses[0]==r->addresses[1]))) {
            DryosDebugMsg(0,15,"RPSS114 GRANT_BAD gen=%d n=%d a=%x b=%x code=%x",
                generation,r->count,r->addresses[0],r->addresses[1],r->error_code);
            c->state=RP_SS110_QUARANTINE; /* cannot safely guess ownership */
            continue;
        }
        /* A-only is the proven output-buffer lease contract. An unexpected
         * valid B grant can be freed, but never exposed to the module. */
        int eligible=(r->count==1u && r->addresses[0]==RP_SS103_A_RAW &&
              (rp_ss103_r32(RP_SS103_OCC_WORD1)&RP_SS103_FLAG_A));
        if (!eligible || c->cancelled) {
            DryosDebugMsg(0,15,"RPSS114 NO_LEASE gen=%d n=%d cancelled=%d",
                          generation,r->count,c->cancelled);
            int rc=rp_ss114_release_core(r,generation);
            c->state=rc ? RP_SS110_QUARANTINE : RP_SS110_FAILED;
            continue;
        }
        /* Offer / acceptance handshake closes the v110 deadline race.
         * HELD means the public acquire() irrevocably accepted this lease. */
        c->state=RP_SS114_OFFERED;
        __sync_synchronize();
        while (c->state==RP_SS114_OFFERED || c->state==RP_SS110_HELD) {
            if (c->bad_callbacks) c->state=RP_SS110_QUARANTINE;
            else if (c->cancelled)
                __sync_bool_compare_and_swap(&c->state,RP_SS114_OFFERED,
                                             RP_SS110_RELEASE_REQUEST);
            msleep(RP_SS110_POLL_MS);
        }
        if (c->state!=RP_SS110_RELEASE_REQUEST) continue;
        int rc=rp_ss114_release_core(r,generation);
        if (rc) {
            c->state=RP_SS110_QUARANTINE;
            continue;
        }
        if (c->cancelled) {
            c->state=RP_SS110_FAILED;
            DryosDebugMsg(0,15,"RPSS114 LATE_RELEASED gen=%d",generation);
        } else if (c->release_timeout_latch) {
            c->state=RP_SS110_QUARANTINE;
            DryosDebugMsg(0,15,"RPSS114 FREE_LATE gen=%d restart_required=1",generation);
        } else {
            c->state=RP_SS110_RELEASED;
            DryosDebugMsg(0,15,"RPSS114 RELEASED gen=%d ready_next=1",generation);
        }
    }
}

/* Public API retained to avoid changing the proven raw_vidx v113.1 writer.
 * A new session is possible only after the prior native free completed,
 * restored the A/B occupancy flags, and the next preflight succeeds. */
__attribute__((used,externally_visible))
int rp_ssraw_v110_acquire(uint32_t *base, uint32_t *bytes)
{
    struct rp_ss110_core *c=&rp_ss110;
    if (!base || !bytes) return -10;
    *base=*bytes=0;
    if (c->bad_callbacks) return -15;
    if (c->release_timeout_latch) return -17;
    if (!rp_ss103_preflight()) return -11;
    uint32_t state=c->state;
    if (state!=RP_SS110_UNUSED && state!=RP_SS110_RELEASED) return -12;
    if (!__sync_bool_compare_and_swap(&c->state,state,RP_SS114_PREPARE)) return -12;
    if (state==RP_SS110_UNUSED &&
        !__sync_bool_compare_and_swap(&rp_ss103_attempted,0,1)) {
        c->state=RP_SS110_QUARANTINE; /* v103 has native ownership */
        return -12;
    }
    uint32_t generation=c->generations_used;
    if (generation>=RP_SS114_MAX_SESSIONS) {
        c->state=RP_SS110_QUARANTINE;
        DryosDebugMsg(0,15,"RPSS114 max generations=%d; reboot",RP_SS114_MAX_SESSIONS);
        return -16;
    }
    struct rp_ss114_round *r=&rp_ss114_rounds[generation];
    /* This generation has never been used; its callback counters are zero. */
    r->free_cookie[0].generation=generation;
    r->free_cookie[0].slot=0u;
    r->free_cookie[1].generation=generation;
    r->free_cookie[1].slot=1u;
    c->active_generation=generation;
    c->generations_used=generation+1u;
    c->cancelled=0u;
    if (!c->task_started) {
        uint32_t tid=task_create("RPSS114",0x1c,0x3000,(void *)rp_ss114_task,0);
        if (!tid) {
            c->state=RP_SS110_QUARANTINE;
            DryosDebugMsg(0,15,"RPSS114 TASK_FAIL");
            return -13;
        }
        c->task_started=1u;
    }
    __sync_synchronize();
    c->state=RP_SS110_START;
    uint32_t start=(uint32_t)get_ms_clock();
    while ((uint32_t)get_ms_clock()-start<RP_SS110_WAIT_MS) {
        if (__sync_bool_compare_and_swap(&c->state,RP_SS114_OFFERED,
                                         RP_SS110_HELD)) {
            *base=RP_SS103_A_RAW;
            *bytes=RP_SS103_BANK_BYTES;
            DryosDebugMsg(0,15,"RPSS114 HELD gen=%d addr=%x bytes=%x",
                          generation,*base,*bytes);
            return 0;
        }
        if (c->state==RP_SS110_FAILED || c->state==RP_SS110_QUARANTINE)
            return -14;
        msleep(RP_SS110_POLL_MS);
    }
    /* No caller receives a lease once abandoned; any late valid grant is
     * freed by the permanent worker using its immutable generation cookie. */
    c->cancelled=1u;
    __sync_synchronize();
    __sync_bool_compare_and_swap(&c->state,RP_SS114_OFFERED,
                                 RP_SS110_RELEASE_REQUEST);
    DryosDebugMsg(0,15,"RPSS114 TIMEOUT gen=%d native job NOT cancelled",generation);
    return -2;
}

__attribute__((used,externally_visible))
int rp_ssraw_v110_release(void)
{
    struct rp_ss110_core *c=&rp_ss110;
    if (!__sync_bool_compare_and_swap(&c->state,RP_SS110_HELD,
                                      RP_SS110_RELEASE_REQUEST)) return -1;
    uint32_t start=(uint32_t)get_ms_clock();
    while ((uint32_t)get_ms_clock()-start<RP_SS110_WAIT_MS) {
        if (c->state==RP_SS110_RELEASED) return 0;
        if (c->state==RP_SS110_QUARANTINE || c->state==RP_SS110_FAILED)
            return -3;
        msleep(RP_SS110_POLL_MS);
    }
    c->release_timeout_latch=1u;
    __sync_synchronize();
    return -2; /* free may still be pending; no further lease this boot */
}

__attribute__((used,externally_visible))
uint32_t rp_ssraw_v110_state(void)
{
    return rp_ss110.state;
}

/* Optional diagnostics for a future per-session log/UI. */
__attribute__((used,externally_visible))
uint32_t rp_ssraw_v114_sessions_used(void)
{
    return rp_ss110.generations_used;
}
