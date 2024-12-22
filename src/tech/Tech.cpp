#include "Tech.h"
#include "XC7Tech.h"

using namespace tech;

CombDelays Tech::comb_delays;
std::multimap<std::string,std::string> Tech::clocked_ports;
std::multimap<std::string,std::string> Tech::buffers_ports;
