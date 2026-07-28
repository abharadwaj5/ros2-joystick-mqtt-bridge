// Copyright 2026 Abhay Bharadwaj
// Licensed under the MIT License. See LICENSE file in the project root.

#include <gtest/gtest.h>
#include "remote_robot_bridge/fallback_logic.hpp"

// Standard limits used across the tests: 0.5 m/s, 1.0 rad/s, 1000 ms timeout.
static const BridgeLimits LIM{0.5, 1.0, 1000};

// A helper that returns a fully healthy set of inputs (robot allowed to move).
// Each test then breaks ONE thing and checks the decision reacts correctly.
static BridgeInputs healthy()
{
    BridgeInputs in;
    in.connected       = true;
    in.have_command    = true;
    in.cmd_age_ms      = 50;      // fresh
    in.heartbeat_age_ms = 50;     // fresh
    in.valid           = true;
    in.deadman         = true;    // held
    in.linear          = 0.3;
    in.angular         = 0.2;
    return in;
}

// When everything is healthy, the robot moves and reports "ok".
TEST(FallbackLogic, HealthyAllowsMotion)
{
    auto d = decide(healthy(), LIM);
    EXPECT_FALSE(d.fallback);
    EXPECT_EQ(d.reason, "ok");
    EXPECT_DOUBLE_EQ(d.linear, 0.3);
    EXPECT_DOUBLE_EQ(d.angular, 0.2);
}

// Each of the six fallback conditions, one per test.

TEST(FallbackLogic, DisconnectedStops)
{
    auto in = healthy();
    in.connected = false;
    auto d = decide(in, LIM);
    EXPECT_TRUE(d.fallback);
    EXPECT_EQ(d.reason, "mqtt_disconnected");
    EXPECT_DOUBLE_EQ(d.linear, 0.0);
}

TEST(FallbackLogic, NoCommandYetStops)
{
    auto in = healthy();
    in.have_command = false;
    auto d = decide(in, LIM);
    EXPECT_TRUE(d.fallback);
    EXPECT_EQ(d.reason, "no_command_yet");
}

TEST(FallbackLogic, OldCommandStops)
{
    auto in = healthy();
    in.cmd_age_ms = 1500;   // older than the 1000 ms timeout
    auto d = decide(in, LIM);
    EXPECT_TRUE(d.fallback);
    EXPECT_EQ(d.reason, "command_old");
}

TEST(FallbackLogic, OldHeartbeatStops)
{
    auto in = healthy();
    in.heartbeat_age_ms = 1500;
    auto d = decide(in, LIM);
    EXPECT_TRUE(d.fallback);
    EXPECT_EQ(d.reason, "heartbeat_old");
}

TEST(FallbackLogic, InvalidCommandStops)
{
    auto in = healthy();
    in.valid = false;
    auto d = decide(in, LIM);
    EXPECT_TRUE(d.fallback);
    EXPECT_EQ(d.reason, "invalid_command");
}

TEST(FallbackLogic, DeadmanReleasedStops)
{
    auto in = healthy();
    in.deadman = false;
    auto d = decide(in, LIM);
    EXPECT_TRUE(d.fallback);
    EXPECT_EQ(d.reason, "deadman_is_false");
}

// The order matters: the FIRST failing check should name the reason.
// Here connection is down AND deadman is released; it must report the
// connection, because that is checked first.
TEST(FallbackLogic, FirstFailureWins)
{
    auto in = healthy();
    in.connected = false;
    in.deadman   = false;
    auto d = decide(in, LIM);
    EXPECT_EQ(d.reason, "mqtt_disconnected");
}

// Clamping: a valid but absurd command must be capped to the limits.
TEST(FallbackLogic, ClampsExcessiveValues)
{
    auto in = healthy();
    in.linear  = 99.0;    // way over 0.5
    in.angular = -99.0;   // way under -1.0
    auto d = decide(in, LIM);
    EXPECT_FALSE(d.fallback);
    EXPECT_DOUBLE_EQ(d.linear, 0.5);    // capped
    EXPECT_DOUBLE_EQ(d.angular, -1.0);  // capped
}