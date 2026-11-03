#pragma once
#include "cleaner/geometry_types.hpp"
#include <chrono>
#include <vector>


struct Header {
    std::chrono::system_clock::time_point stamp_;
    std::string frame_id_;
};

struct Twist2D {
    double linear_x_ = 0.0; // m/s
    double linear_y_ = 0.0;
    double angular_z_ = 0.0; // rad/s
};

struct RobotState {
    Header header_; // "map"
    // State variables defined by the selected robot model.
    Pose2D pose_;

    // Frame in which twist is expressed --> twist is expreseed in body frame because motors dont know about world frame
    std::string child_frame_id_ = "base_link";
    Twist2D twist_;
};

struct ControlCommand {
    Header header_;  // Command frame, e.g. "base_link"
    Twist2D twist_;
};

enum class ControlStatus {
    SUCCESS,
    FAILURE
};

struct ControlResult {
    ControlCommand command_;
    ControlStatus status_;
};


enum class ExecutionStatus {
    IDLE,
    RUNNING,
    COMPLETED,
    BLOCKED,
    FAILED,
};

struct ExecutionResult {
    ControlCommand command_;
    ExecutionStatus status_;
};