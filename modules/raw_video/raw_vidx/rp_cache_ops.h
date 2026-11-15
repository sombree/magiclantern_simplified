#ifndef _RAW_VIDX_RP_CACHE_OPS_H_
#define _RAW_VIDX_RP_CACHE_OPS_H_

#include <stdint.h>

/* Canon EOS RP 1.6.0 native cache-clean routine, resolved by the platform
 * stubs.S (E05A9F08).  Cache coherency for DMA is not an MMU ROM remap. */
extern void dcache_clean(uint32_t addr, uint32_t size);

#endif
