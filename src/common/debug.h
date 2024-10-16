#pragma once

#define PNR_LOG(module, a...)  { std::println(stdout, module "  " a); }
#define PNR_LOG1(module, a...)  { std::println(stdout, module "  " a); }
#define PNR_LOG2(module, a...)  { std::println(stdout, module "  " a); }
#define PNR_LOG3(module, a...)  { std::println(stdout, module "  " a); }
#define PNR_DEBUG(a...)  { std::print(stderr, a); }
#define PNR_DEBUG1(module, a...) { std::print(stderr, module "  " a); }
#define PNR_DEBUG2(module, a...) { std::print(stderr, module "  " a); }
#define PNR_DEBUG3(module, a...) {} // { std::print(stderr, module "  " a); }
#define PNR_WARNING(a...) { std::print(stderr, "WARNING: " a); }

#include "Formatting.h"
