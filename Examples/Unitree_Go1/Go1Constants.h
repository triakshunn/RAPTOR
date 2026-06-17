/// TODO: Values not verified.
#ifndef GO1_CONSTANTS_H
#define GO1_CONSTANTS_H

namespace RAPTOR {
namespace Go1 {

constexpr int NUM_JOINTS = 12;

constexpr double JOINT_LIMITS_LOWER[NUM_JOINTS] = {
    -49.45, -39.3, -161.46, // FR (hip, thigh, calf)
    -49.45, -39.3, -161.46, // FL
    -49.45, -39.3, -161.46, // RR
    -49.45, -39.3, -161.46  // RL
};

constexpr double JOINT_LIMITS_UPPER[NUM_JOINTS] = {
    49.45, 257.9, -50.88, // FR
    49.45, 257.9, -50.88, // FL
    49.45, 257.9, -50.88, // RR
    49.45, 257.9, -50.88  // RL
};

constexpr double VELOCITY_LIMITS_LOWER[NUM_JOINTS] = {
    -1724.6, -1724.6, -1149.3, -1724.6, -1724.6, -1149.3,
    -1724.6, -1724.6, -1149.3, -1724.6, -1724.6, -1149.3};

constexpr double VELOCITY_LIMITS_UPPER[NUM_JOINTS] = {
    1724.6, 1724.6, 1149.3, 1724.6, 1724.6, 1149.3,
    1724.6, 1724.6, 1149.3, 1724.6, 1724.6, 1149.3};

constexpr double ACCELERATION_LIMITS_LOWER[NUM_JOINTS] = {
    -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0,
    -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0};

constexpr double ACCELERATION_LIMITS_UPPER[NUM_JOINTS] = {
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};

constexpr double TORQUE_LIMITS_LOWER[NUM_JOINTS] = {
    -23.7, -23.7, -35.55, -23.7, -23.7, -35.55,
    -23.7, -23.7, -35.55, -23.7, -23.7, -35.55};

constexpr double TORQUE_LIMITS_UPPER[NUM_JOINTS] = {
    23.7, 23.7, 35.55, 23.7, 23.7, 35.55, 23.7, 23.7, 35.55, 23.7, 23.7, 35.55};

constexpr double GRAVITY = -9.81; // m/s^2

}; // namespace Go1
}; // namespace RAPTOR

#endif // GO1_CONSTANTS_H