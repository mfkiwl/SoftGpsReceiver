#pragma once

#include <cstdint>
#include <vector>

namespace gnss {

// GLONASS L1OF ranging code, the same for every satellite (ICD Edition 5.1, section 3.3.2.2)
class GlonassCode
{
public:
    // +1/-1 chips of the 511-chip code: logic 0 -> +1, logic 1 -> -1
    static std::vector<int8_t> chips();
};

} // namespace gnss
