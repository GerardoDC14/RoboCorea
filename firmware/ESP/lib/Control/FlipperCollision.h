#pragma once
#include "config.h"
#include <cmath>

// ─── Flipper collision avoidance (dynamic joint limits) ──────────────────────
// The front and rear flipper on the SAME side collide when both point toward the
// middle of the robot: the front leaning back (~180°) and the rear leaning
// forward (~0°). Forbid exactly that joint state — a flipper may not enter its
// danger arc while its same-side partner is already in its own danger arc;
// otherwise both spin freely. Left (FL/RL) and right (FR/RR) are independent.
//
// Each danger arc is CENTER ± HALFWIDTH on the wrapped [0,360) angle; all four
// knobs live in config.h (FLIPPER_COLLISION_*). Defaults: FRONT 180±60 =
// [120,240], REAR 0±60 = [300,60] (the arc through 0°).
//
// This operates on the COMMANDED targets, so the firmware never drives the two
// flippers into each other. It does not react to a flipper forced into its arc
// purely by external load (that motion is not commanded).
//
// Flipper index order matches the rest of the firmware: 0=FL 1=FR 2=RL 3=RR;
// the front↔rear pairs are FL↔RL (left) and FR↔RR (right).

namespace FlipperCollision {

// True if `deg` (any continuous angle) is within `half` of `center`, wrapped.
inline bool inArc(float deg, float center, float half) {
    float d = fmodf(deg - center, 360.0f);
    if (d < 0.0f)   d += 360.0f;        // → [0,360)
    if (d > 180.0f) d  = 360.0f - d;    // → shortest distance [0,180]
    return d <= half;
}

inline bool frontInArc(float deg) {
    return inArc(deg, FLIPPER_COLLISION_FRONT_CENTER_DEG,
                      FLIPPER_COLLISION_FRONT_HALFWIDTH_DEG);
}
inline bool rearInArc(float deg) {
    return inArc(deg, FLIPPER_COLLISION_REAR_CENTER_DEG,
                      FLIPPER_COLLISION_REAR_HALFWIDTH_DEG);
}

// For one side (front index f, rear index r), hold either flipper just shy of its
// danger arc when committing the integrated move would put BOTH into their arcs
// at once. Only an allowed→forbidden transition is blocked: an already-forbidden
// pair (boot / manual / load) may always move, so it can recover.
//   prev = last committed targets, cand = freshly integrated targets (modified).
inline void applySide(int f, int r, const float prev[4], float cand[4]) {
    if (frontInArc(prev[f]) && rearInArc(prev[r])) return;       // already forbidden → free
    if (!(frontInArc(cand[f]) && rearInArc(cand[r]))) return;    // candidate pair is fine
    if (!frontInArc(prev[f])) cand[f] = prev[f];   // front was clear → keep it clear
    if (!rearInArc(prev[r]))  cand[r] = prev[r];   // rear was clear → keep it clear
}

}  // namespace FlipperCollision
