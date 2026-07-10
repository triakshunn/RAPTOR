#ifndef CONDITION_NUMBER_OPTIMIZER_H
#define CONDITION_NUMBER_OPTIMIZER_H

#include "Go1Constants.h"

#include "Optimizer.h"

#include "FixedFrequencyFourierCurves.h"
#include "RegressorInverseDynamics.h"

#include "EndEffectorRegressorConditionNumber.h"

#include "AccelerationLimits.h"
#include "JointLimits.h"
#include "KinovaCustomizedConstraints.h"
#include "TorqueLimits.h"
#include "TrajectoryTerminalConstraints.h"
#include "VelocityLimits.h"

namespace RAPTOR {
namespace Go1 {

class PayloadExcitingTrajectoryGenerator : public Optimizer {
public:
  using Model = pinocchio::Model;
  using Vec3 = Eigen::Vector3d;
  using VecX = Eigen::VectorXd;
  using MatX = Eigen::MatrixXd;

  /** Default constructor */
  PayloadExcitingTrajectoryGenerator() = default;

  /** Default destructor */
  ~PayloadExcitingTrajectoryGenerator() = default;

  // [set_parameters]
  bool set_parameters(const VecX &x0_input, const double T_input,
                      const int N_input, const int degree_input,
                      const double base_frequency_input, const VecX &q0_input,
                      const VecX &q_d0_input, const Model &model_input,
                      const std::vector<Vec3> &boxCenters,
                      const std::vector<Vec3> &boxOrientations,
                      const std::vector<Vec3> &boxSizes,
                      const VecX &joint_limits_buffer_input,
                      const VecX &velocity_limits_buffer_input,
                      const VecX &torque_limits_buffer_input,
                      const bool include_gripper_or_not = true,
                      const double collison_buffer_input = 0.0,
                      Eigen::VectorXi jtype_input = Eigen::VectorXi(0));

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the NLP */
  bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag,
                    IndexStyleEnum &index_style) final override;

  /**@name Methods to block default compiler methods.
   *
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   */
  //@{
  PayloadExcitingTrajectoryGenerator(
      const PayloadExcitingTrajectoryGenerator &);

  PayloadExcitingTrajectoryGenerator &
  operator=(const PayloadExcitingTrajectoryGenerator &);

  std::shared_ptr<Trajectories> trajPtr_;

  std::shared_ptr<RegressorInverseDynamics> ridPtr_;
};

}; // namespace Go1
}; // namespace RAPTOR

#endif // CONDITION_NUMBER_OPTIMIZER_H