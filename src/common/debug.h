#pragma once

#define PNR_LOG(module, a...)  { std::print(stdout, "\n" module "  " a); }
#define PNR_LOG1(module, a...)  { std::print(stdout, "\n" module "      " a); }
#define PNR_LOG2(module, a...)  { std::print(stdout, "\n" module "          " a); }
#define PNR_LOG3(module, a...)   { std::print(stdout, a); }
#define PNR_DEBUG(module, a...)  { std::print(stdout, module "  " a); }
#define PNR_DEBUG1(module, a...) { std::print(stdout, module "  " a); }
#define PNR_DEBUG2(module, a...) { std::print(stdout, module "  " a); }
#define PNR_DEBUG3(a...) {} // { std::print(stdout, module "  " a); }
#define PNR_WARNING(a...) { std::print(stdout, "\nWARNING: " a); }
#define PNR_ERROR(a...) { std::print(stderr, "\n\nERROR: " a); }

#include "Formatting.h"
