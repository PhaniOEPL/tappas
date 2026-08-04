/**
 * Class-confusion policy for the JDE tracker.
 *
 * WHY THIS EXISTS
 * ---------------
 * The association path enforces class consistency. Treating the detector's
 * per-frame label as an identity constraint is wrong for classes the detector
 * genuinely confuses: a single frame of doubt vetoes the correct match, the
 * detection falls through to track creation, and the object ends up holding two
 * ids - one per label - each coasting on the frames the other wins. Because the
 * flicker period (a few frames) is far shorter than keep_tracked_frames, neither
 * track ever expires and the duplicate is permanent.
 *
 * The fix has two halves, and both live here:
 *
 *   1. Class is treated as a NOISY OBSERVATION, not an identity. A track
 *      accumulates class evidence over its lifetime (STrack::vote_class) and
 *      reports the argmax, so a few frames of contrary labelling cannot move it.
 *
 *   2. For pairs the detector actually confuses, the hard veto becomes a COST
 *      PENALTY, so overwhelming geometric evidence can override the label
 *      disagreement. Adding a penalty p is arithmetically identical to raising
 *      the IOU requirement for that pair alone: with cost = 1 - IOU and a match
 *      threshold of 0.8, a penalty of 0.3 means a confusable-class match needs
 *      IOU >= 0.5 where a same-class match needs only IOU >= 0.2. Unrelated
 *      classes keep the outright veto, so a person track still cannot absorb a
 *      car detection.
 *
 * TAXONOMY NOTE
 * -------------
 * The ids below are the nv_imx label set (see
 * libs/postprocesses/common/labels/nv_imx.hpp). They are duplicated here rather
 * than included so the tracker does not depend on a post-process header. If the
 * label set changes, both must be updated.
 *
 * Only birds/drones are listed as confusable, and that is deliberate: it is the
 * only pair observed to flicker in practice. Drone is NOT confused with
 * helicopter, flight or fighterflight, so those keep the hard veto. Do not add
 * pairs speculatively - every pair added is a pair the tracker may merge under
 * strong overlap.
 */

#pragma once

#include <map>

// ---------------------------------------------------------------------------
// Class voting (STrack::vote_class)
// ---------------------------------------------------------------------------

// Per-observation decay applied to accumulated class scores. 0.97 gives a
// half-life of ~23 observations (~0.4s at 60fps). Long enough that a short
// mislabel run cannot move the argmax, short enough that a genuine
// reclassification settles in well under a second.
#define CLASS_VOTE_DECAY (0.97f)

// A challenging class must beat the incumbent by this factor before the track
// relabels. Pure hysteresis - stops the label oscillating when two classes score
// almost equally.
#define CLASS_SWITCH_MARGIN (1.2f)

// Extra margin required to relabel AWAY FROM a sticky class. Mission-asymmetric:
// calling a drone a bird is far more costly than calling a bird a drone, so a
// track that has established itself as a drone demands much stronger evidence
// before it is downgraded. Applies only to the classes in is_sticky_class().
#define STICKY_CLASS_EXTRA_MARGIN (2.0f)

// ---------------------------------------------------------------------------
// Class gating (JDETracker::fuse_motion_custom)
// ---------------------------------------------------------------------------

// Cost added to a confusable-class pair instead of vetoing it outright. Cost is
// 1 - IOU, so this raises the effective IOU requirement for that pair by this
// amount. Keep it well below the match threshold or the pair becomes a de-facto
// veto again.
#define CLASS_MISMATCH_PENALTY (0.3f)

// ---------------------------------------------------------------------------
// nv_imx taxonomy ids referenced by the policy
// ---------------------------------------------------------------------------

#define NV_IMX_CLASS_BIRDS (7)
#define NV_IMX_CLASS_DRONES (10)

/**
 * @brief Whether two DIFFERENT class ids are ones the detector is known to
 *        confuse, and may therefore describe the same physical object.
 *
 *        Returns false for equal ids - callers handle the same-class case before
 *        reaching here.
 */
inline bool classes_are_confusable(int class_a, int class_b)
{
    if (class_a == class_b)
        return false;

    // Birds <-> drones. Small, distant, similar silhouette; the only pair
    // observed to flicker on real footage.
    if ((class_a == NV_IMX_CLASS_BIRDS && class_b == NV_IMX_CLASS_DRONES) ||
        (class_a == NV_IMX_CLASS_DRONES && class_b == NV_IMX_CLASS_BIRDS))
        return true;

    return false;
}

/**
 * @brief Classes that resist being relabelled away from, once established.
 *        See STICKY_CLASS_EXTRA_MARGIN for the rationale.
 */
inline bool is_sticky_class(int class_id)
{
    return (class_id == NV_IMX_CLASS_DRONES);
}

/**
 * @brief The evidence ratio a challenger must exceed to take over the label of a
 *        track currently classified as current_class.
 */
inline float class_switch_margin(int current_class)
{
    if (is_sticky_class(current_class))
        return CLASS_SWITCH_MARGIN * STICKY_CLASS_EXTRA_MARGIN;

    return CLASS_SWITCH_MARGIN;
}
