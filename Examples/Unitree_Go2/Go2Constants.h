// Limits taken from URDF 
#ifndef GO2_CONSTANTS_H
#define GO2_CONSTANTS_H

namespace RAPTOR {
namespace Go2 {

constexpr int NUM_JOINTS = 3;

constexpr double JOINT_LIMITS_LOWER[NUM_JOINTS] = {
    -60.0,
    -90.0,
    -156.0, // FR (hip, thigh, calf) (-1.0472, -1.5708, -2.7227) 
};

constexpr double JOINT_LIMITS_UPPER[NUM_JOINTS] = {
    60,
    200,
    -48, // FR (1.0472, 3.4907, -0.83776)
};

constexpr double VELOCITY_LIMITS_LOWER[NUM_JOINTS] = {-1724.6, -1724.6,
                                                      -899.5};

constexpr double VELOCITY_LIMITS_UPPER[NUM_JOINTS] = {1724.6, 1724.6, 899.5};

constexpr double ACCELERATION_LIMITS_LOWER[NUM_JOINTS] = {-10000.0, -10000.0,
                                                          -10000.0};

constexpr double ACCELERATION_LIMITS_UPPER[NUM_JOINTS] = {10000.0, 10000.0,
                                                          10000.0};

constexpr double TORQUE_LIMITS_LOWER[NUM_JOINTS] = {-23.7, -23.7, -45.4};

constexpr double TORQUE_LIMITS_UPPER[NUM_JOINTS] = {23.7, 23.7, 45.4};

constexpr double GRAVITY = -9.81; // m/s^2

// ── Nominal joint positions (radians) ────────────────────────────────────────
// Joint order: [FR_hip, FR_thigh, FR_calf,
//               FL_hip, FL_thigh, FL_calf,
//               RR_hip, RR_thigh, RR_calf,
//               RL_hip, RL_thigh, RL_calf]
//
// Standard standing pose (body ~0.265 m above ground).
// Foot Z ≈ -0.265 m relative to body origin.
// Used as the excitation center for friction SysID.
constexpr double STAND_POSITIONS[NUM_JOINTS] = {
    0.0,
    0.9,
    -1.8, // FR
};

// Prone (lying flat) pose — legs folded tight against the belly.
// Foot Z ≈ +0.037 m (feet above body center) → body rests on its belly.
// Verified via Pinocchio FK: all joints within URDF limits.
// Use this as q_nominal for legs NOT being excited during inertial SysID,
// so the robot can lay flat while only one leg moves.
constexpr double PRONE_POSITIONS[NUM_JOINTS] = {
    0.0,
    3.5,
    -2.8, // FR
};

}; // namespace Go2
}; // namespace RAPTOR

#endif // GO2_CONSTANTS_H