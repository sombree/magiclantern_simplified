# define the module name - make sure name is max 8 characters
MODULE_NAME = raw_vidx
MODULE_OBJS = $(BUILD_DIR)/raw_vid.o \
              $(BUILD_DIR)/worker.o \
              $(BUILD_DIR)/rp_adaptive_writer.o \
              $(BUILD_DIR)/rp_pressure_v112.o \
              $(BUILD_DIR)/rp_pressure_v113.o \
              $(BUILD_DIR)/mlv_3.o \
              $(BUILD_DIR)/event_pusher.o \
              $(BUILD_DIR)/rp_backend.o \
              $(BUILD_DIR)/rp_crx.o \
              $(BUILD_DIR)/rp_w3_matrix.o \
              $(BUILD_DIR)/rp_w3_transport.o \
              $(BUILD_DIR)/rp_hot_thumb.o
TOP_DIR = ../../..

# include modules environment
include $(TOP_DIR)/modules/Makefile

# Keep the normal raw_vidx module objects on the standard ML module ABI
# (ARMv5TE/ARM state).  This is the same architecture used by ordinary ML
# modules and avoids turning a large multi-object .mo into a Thumb module.
#
# EOS RP-specific control code is ARMv7-A but remains in ARM state.  The only
# Thumb entry is the existing Canon ch3 Complete callback, installed by a
# writable RAM callback-table swap (not a ROM function hook or MMU patch).
# The Thumb callback object never calls ML/DryOS code.
CFLAGS_ARCHLESS = $(filter-out -march=armv7-r -march=armv7-a -march=armv7 -march=armv5te -mfloat-abi=%,$(CFLAGS))

$(BUILD_DIR)/rp_backend.o: rp_backend.c rp_backend.h rp_hot_shared.h rp_cache_ops.h
	$(CC) $(CFLAGS_ARCHLESS) -marm -march=armv7-a -mfloat-abi=soft $(CDEPS_FLAGS) -c -o $@ $<

$(BUILD_DIR)/rp_w3_matrix.o: rp_w3_matrix.c rp_w3_matrix.h rp_backend.h
	$(CC) $(CFLAGS_ARCHLESS) -marm -march=armv7-a -mfloat-abi=soft $(CDEPS_FLAGS) -c -o $@ $<

$(BUILD_DIR)/rp_crx.o: rp_crx.c rp_crx.h rp_crx_format.h
	$(CC) $(CFLAGS_ARCHLESS) -marm -march=armv7-a -mfloat-abi=soft $(CDEPS_FLAGS) -c -o $@ $<

$(BUILD_DIR)/rp_hot_thumb.o: rp_hot_thumb.c rp_hot_shared.h
	$(CC) $(CFLAGS_ARCHLESS) -mthumb -march=armv7-a -mfloat-abi=soft $(CDEPS_FLAGS) -c -o $@ $<

.DEFAULT_GOAL := $(BUILD_DIR)/module_complete
