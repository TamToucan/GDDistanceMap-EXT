#ifndef UTIL_GAME_TIME_H
#define UTIL_GAME_TIME_H

#include "Range.h"

namespace Game {

    typedef float GameTime;

    const float ONE_60FPS_FRAME = (1000.0f/60.f);

    typedef Util::Range<GameTime> GameTimeRange;

} // namespace

#endif
