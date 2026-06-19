#include "EndEffectorRegressorConditionNumber.h"

namespace RAPTOR {

EndEffectorRegressorConditionNumber::EndEffectorRegressorConditionNumber(
    std::shared_ptr<Trajectories> &trajPtr_input,
    std::shared_ptr<RegressorInverseDynamics> &ridPtr_input)
    : trajPtr_(trajPtr_input), ridPtr_(ridPtr_input) {
  initialize_memory(trajPtr_->varLength);
}

void EndEffectorRegressorConditionNumber::compute(const VecX &z,
                                                  bool compute_derivatives,
                                                  bool compute_hessian) {
  if (is_computed(z, compute_derivatives, compute_hessian)) {
    return;
  }

  if (compute_hessian) {
    throw std::invalid_argument("EndEffectorRegressorConditionNumber does not "
                                "support hessian computation");
  }

  // this will be called in ridPtr_->compute
  // trajPtr_->compute(z, compute_derivatives, compute_hessian);

  ridPtr_->compute(z, compute_derivatives, compute_hessian);

  // get the regressor corresponding to the end effector
  const int startCol = (ridPtr_->NB - 1) * 10; // changed from 1 to 3
  const MatX &EndeffectorY = ridPtr_->Y.middleCols(
      startCol,
      10); // changed from 10 to 30 for full leg parameter identification

  Eigen::JacobiSVD<MatX> svd(EndeffectorY,
                             Eigen::ComputeThinU | Eigen::ComputeThinV);
  const VecX &singularValues = svd.singularValues();
  const MatX &U = svd.matrixU();
  const MatX &V = svd.matrixV();

  // DEBUG: print column norms and singular values (only once)
  static bool debug_printed = false;
  if (!debug_printed) {
    debug_printed = true;
    std::cout << "\n=== EndeffectorY debug (shape " << EndeffectorY.rows()
              << "x" << EndeffectorY.cols() << ") ===\n";

    std::cout << "EndeffectorY: \n" << EndeffectorY << "\n\n";

    std::cout << "Column L2 norms:\n";
    for (int c = 0; c < EndeffectorY.cols(); c++) {
      std::cout << "  col[" << c << "] = " << EndeffectorY.col(c).norm();
      if (EndeffectorY.col(c).norm() < 1e-10)
        std::cout << "  <-- ZERO";
      std::cout << "\n";
    }
    std::cout << "Singular values (" << singularValues.size() << " total):\n";
    for (int i = 0; i < singularValues.size(); i++) {
      std::cout << "  sigma[" << i << "] = " << singularValues(i);
      if (singularValues(i) < 1e-6)
        std::cout << "  <-- near-zero";
      std::cout << "\n";
    }
    std::cout << "=== end debug ===\n\n";
  }

  const size_t lastRow = singularValues.size() - 1;
  const double &sigmaMax = singularValues(0);
  const double &sigmaMin = singularValues(lastRow);

  // log of the condition number in 2-norm
  // (ratio between the largest and the smallest singular values)
  f = std::log(sigmaMax) - std::log(sigmaMin);

  if (compute_derivatives) {
    // refer to (17) in https://j-towns.github.io/papers/svd-derivative.pdf
    // for analytical gradient of singular values
    for (int i = 0; i < trajPtr_->varLength; i++) {
      const int startCol = (ridPtr_->NB - 1) * 10;
      const MatX &gradEndeffectorY = ridPtr_->pY_pz(i).middleCols(startCol, 10);

      const double gradSigmaMax =
          U.col(0).transpose() * gradEndeffectorY * V.col(0);
      const double gradSigmaMin =
          U.col(lastRow).transpose() * gradEndeffectorY * V.col(lastRow);

      grad_f(i) = gradSigmaMax / sigmaMax - gradSigmaMin / sigmaMin;
    }
  }
}

}; // namespace RAPTOR