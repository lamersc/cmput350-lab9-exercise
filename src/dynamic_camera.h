#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <algorithm>
#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        camera.setCameraCenter(cameraUnadjustedCenter);
        float playerWindowPercentX = camera.worldToWindowPercent(trackedPosWorld).x;

        float lookAheadWorldX = 0.f;
        if (trackedVelocityWorld.x > 0.f) {
            lookAheadWorldX = camera.windowPercentToWorldDist({lookAhead, 0.f}).x;
        } else if (trackedVelocityWorld.x < 0.f) {
            lookAheadWorldX = -camera.windowPercentToWorldDist({lookAhead, 0.f}).x;
        }
        lookaheadState += alpha * (lookAheadWorldX - lookaheadState);

        float targetWindowPercentX = std::clamp(playerWindowPercentX, deadZoneX.min, deadZoneX.max);
        float zoneAlpha = alpha;
        if (!softZoneX.contains(playerWindowPercentX)) {
            targetWindowPercentX = std::clamp(playerWindowPercentX, softZoneX.min, softZoneX.max);
            zoneAlpha = 1.f;
        }
        float deltaWindowPercentX = zoneAlpha * (playerWindowPercentX - targetWindowPercentX);
        sf::Vector2f deltaWorld = camera.windowPercentToWorldDist({deltaWindowPercentX, 0.f});

        camera.setCameraCenter(cameraUnadjustedCenter + deltaWorld +
                               sf::Vector2f{lookaheadState, 0.f});
        cameraUnadjustedCenter += deltaWorld;
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
