#ifndef SOURCE_MACROS_HPP
#define SOURCE_MACROS_HPP
#include <cstring>

#define PROJECT_RELATIVE_PATH(x) std::string(x).substr(strlen(OSRM_PROJECT_DIR) + 1)
#define OSRM_SOURCE_FILE PROJECT_RELATIVE_PATH(__FILE__)

#endif // SOURCE_MACROS_HPP