// Copyright 2026 Abhay Bharadwaj
// Licensed under the MIT License. See LICENSE file in the project root.

#pragma once
#include <algorithm>
#include <cstdint>
#include <string>

// The safety decision, separated from ROS, MQTT and the clock so it can be
// tested on its own. The node measures the ages and reads the shared state;
// this function only reasons about the numbers it is handed.

struct BridgeInputs
{
    bool    connected;          // is the MQTT link up?
    bool    have_command;       // has any command arrived since startup?
    int64_t cmd_age_ms;         // ms since last command (ignored if !have_command)
    int64_t heartbeat_age_ms;   // ms since last heartbeat
    bool    valid;              // was the last command well-formed?
    bool    deadman;            // is the deadman held?
    double  linear;             // requested forward speed, unclamped
    double  angular;            // requested turn rate, unclamped
};

struct BridgeLimits
{
    double  max_linear;
    double  max_angular;
    int64_t timeout_ms;
};

struct BridgeDecision
{
    bool        fallback;       // true = stop, false = drive
    std::string reason;         // why (matches the strings the node reported)
    double      linear;         // final value to publish (0 if fallback)
    double      angular;
};

inline BridgeDecision decide(const BridgeInputs & in, const BridgeLimits & lim)
{
    BridgeDecision d;
    d.fallback = true;
    d.reason   = "ok";
    d.linear   = 0.0;
    d.angular  = 0.0;

    // Same order as the node: first failing check names the reason.
    if (!in.connected) {
        d.reason = "mqtt_disconnected";
    } else if (!in.have_command) {
        d.reason = "no_command_yet";
    } else if (in.cmd_age_ms > lim.timeout_ms) {
        d.reason = "command_old";
    } else if (in.heartbeat_age_ms > lim.timeout_ms) {
        d.reason = "heartbeat_old";
    } else if (!in.valid) {
        d.reason = "invalid_command";
    } else if (!in.deadman) {
        d.reason = "deadman_is_false";
    } else {
        // All checks passed: allow motion, but clamp to the robot's limits.
        d.fallback = false;
        d.reason   = "ok";
        d.linear   = std::clamp(in.linear,  -lim.max_linear,  lim.max_linear);
        d.angular  = std::clamp(in.angular, -lim.max_angular, lim.max_angular);
    }
    return d;
}