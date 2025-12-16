#pragma once

#include <print>

#define CALC_MOD_MASK_INDEX(mod) ((*(const uint16_t*)mod)/8)
#define CALC_MOD_MASK_VALUE(mod) (1<<((*(const uint16_t*)mod)%8))
#define CHECK_MOD_MASK(mod) (debug_module[CALC_MOD_MASK_INDEX(mod)] & CALC_MOD_MASK_VALUE(mod))

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT_LOCATION __FILE__ ":" TOSTRING(__LINE__)

#define PNR_LOG(module, a...)  { std::print(stdout, "\n" module "  " a); fflush(stdout); }
#define PNR_LOG1(module, a...)  { if (debug_level >= 1 /*&& CHECK_MOD_MASK(module)*/) { std::print(stdout, "\n" module "    " a); fflush(stdout); } }
#define PNR_LOG2(module, a...)  { if (debug_level >= 2 && CHECK_MOD_MASK(module)) { std::print(stdout, "\n" module "      " a); fflush(stdout); } }
#define PNR_LOG2_(module, depth, a...)  { if (debug_level >= 2 && CHECK_MOD_MASK(module)) { std::print("\n" module "      "); for(int i=0;i<depth;++i) std::print("  "); std::print(stdout,  a); fflush(stdout); } }
#define PNR_LOG3(module, a...)   { if (debug_level >= 3 && CHECK_MOD_MASK(module)) { std::print(stdout, "\n" module "      " a); fflush(stdout); } }
#define PNR_LOG3_(module, depth, a...)  { if (debug_level >= 3 && CHECK_MOD_MASK(module)) { std::print("\n" module "      "); for(int i=0;i<depth;++i) std::print("  "); std::print(stdout,  a); fflush(stdout); } }
#define PNR_LOG4(module, a...)   { if (debug_level >= 4 && CHECK_MOD_MASK(module)) { std::print(stdout, a); fflush(stdout); } }
#define PNR_DEBUG(module, a...)  { std::print(stdout, module "  " a); }
#define PNR_DEBUG1(module, a...) { std::print(stdout, module "  " a); }
#define PNR_DEBUG2(module, a...) { std::print(stdout, module "  " a); }
#define PNR_DEBUG3(a...) {} // { std::print(stdout, module "  " a); }
#define PNR_WARNING(a...) { std::print(stdout, "\nWARNING: " a); }
#define PNR_ERROR(a...) { std::print(stderr, "\n\nERROR: " a); }
#define PNR_ASSERT(a, message...) { if (!(a)) { std::print(stderr, "\n!!! ASSERT at " AT_LOCATION ": " message); stop_if_needed(); } }

#include "formatting.h"


extern int debug_level;
extern uint8_t debug_module[65536/8];
void stop_if_needed();
