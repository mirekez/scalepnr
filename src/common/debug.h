#pragma once

#include <print>

#define PNR_LOG(module, a...)  { std::print(stdout, "\n" module "  " a); fflush(stdout); }
#define PNR_LOG1(module, a...)  { if (debug_level >= 1) { std::print(stdout, "\n" module "    " a); fflush(stdout); } }
#define PNR_LOG2(module, a...)  { if (debug_level >= 2) { std::print(stdout, "\n" module "      " a); fflush(stdout); } }
#define PNR_LOG2_(module, depth, a...)  { if (debug_level >= 2) { std::print("\n" module "      "); for(int i=0;i<depth;++i) std::print("  "); std::print(stdout,  a); fflush(stdout); } }
#define PNR_LOG3(module, a...)   { if (debug_level >= 3) { std::print(stdout, a); fflush(stdout); } }
#define PNR_DEBUG(module, a...)  { std::print(stdout, module "  " a); }
#define PNR_DEBUG1(module, a...) { std::print(stdout, module "  " a); }
#define PNR_DEBUG2(module, a...) { std::print(stdout, module "  " a); }
#define PNR_DEBUG3(a...) {} // { std::print(stdout, module "  " a); }
#define PNR_WARNING(a...) { std::print(stdout, "\nWARNING: " a); }
#define PNR_ERROR(a...) { std::print(stderr, "\n\nERROR: " a); }
#define PNR_ASSERT(a, message...) { if (!(a)) { std::print(stderr, "!!! ASSERT at {}:{}", __FILE__, __LINE__); std::print(message); } }

#include "formatting.h"

extern int debug_level;
