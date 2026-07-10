/// TODO: Values not changed for Go2 specific to do
#ifndef GO2_CONSTANTS_H
#define GO2_CONSTANTS_H

namespace RAPTOR {
namespace Go2 {

constexpr int NUM_JOINTS = 3;

constexpr double JOINT_LIMITS_LOWER[NUM_JOINTS] = {
    -49.45,
    -39.3,
    -161.46, // FR (hip, thigh, calf) (-0.863, -.686, -2.818)
};

constexpr double JOINT_LIMITS_UPPER[NUM_JOINTS] = {
    49.45,
    257.9,
    -50.88, // FR (0.863, 4.501, -0.888)
};

constexpr double VELOCITY_LIMITS_LOWER[NUM_JOINTS] = {-1724.6, -1724.6,
                                                      -1149.3};

constexpr double VELOCITY_LIMITS_UPPER[NUM_JOINTS] = {1724.6, 1724.6, 1149.3};

constexpr double ACCELERATION_LIMITS_LOWER[NUM_JOINTS] = {-10000.0, -10000.0,
                                                          -10000.0};

constexpr double ACCELERATION_LIMITS_UPPER[NUM_JOINTS] = {10000.0, 10000.0,
                                                          10000.0};

constexpr double TORQUE_LIMITS_LOWER[NUM_JOINTS] = {-23.7, -23.7, -35.55};

constexpr double TORQUE_LIMITS_UPPER[NUM_JOINTS] = {23.7, 23.7, 35.55};

constexpr double GRAVITY = -9.81; // m/s^2

// ── Nominal joint positions (radians) ────────────────────────────────────────
// Joint order: [FR_hip, FR_thigh, FR_calf,
//               FL_hip, FL_thigh, FL_calf,
//               RR_hip, RR_thigh, RR_calf,
//               RL_hip, RL_thigh, RL_calf]
//
// URDF limits (radians, from go1.urdf):
//   hip   (abduction): lower=-0.863  upper=+0.863
//   thigh (pitch):     lower=-0.686  upper=+4.501
//   calf  (pitch):     lower=-2.818  upper=-0.888

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