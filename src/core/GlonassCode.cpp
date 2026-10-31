#include "GlonassCode.h"
#include "Constants.h"

namespace gnss {

std::vector<int8_t> GlonassCode::chips()
{
    // 9-stage register, 1 + x^5 + x^9, all ones at the start, output taken from stage 7
    int reg[10];
    for (int& r : reg) r = 1;
    std::vector<int8_t> c(kGlonassCodeLength);
    for (int8_t& chip : c) {
        chip = reg[7] ? -1 : 1;
        const int fb = reg[5] ^ reg[9];
        for (int i = 9; i > 1; --i) reg[i] = reg[i - 1];
        reg[1] = fb;
    }
    return c;
}

} // namespace gnss
