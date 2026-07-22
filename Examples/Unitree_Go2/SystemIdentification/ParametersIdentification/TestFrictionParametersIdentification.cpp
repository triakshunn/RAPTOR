/// data logging:
/// 20260721_115700 : 5Hz low pass bw filter  Optimal Solution Found. Objective value: 26303.7
//// 20260721_124600 : Curve fitting  Objective value: 21927
//// 20260722_1700 : Sim data, 25 Hz low pass bw filter Objective value: 0.00210344
//// 20260713_165102 : 25 Hz low pass bw filter Optimal Solution Found. Objective value: 23552.7
#include <ctime>
#include <iomanip>
#include <sstream>
#include "FrictionParametersIdentification.h"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

using namespace RAPTOR;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: ./Go2_SysidFriction_test <leg> <timestamp>\n"
              << "  e.g. ./Go2_SysidFriction_test FR 20250707_1430" << std::endl;
    return 1;
  }
  const std::string leg     = std::string(argv[1]);
  const std::string data_ts = std::string(argv[2]);

  std::time_t t_now = std::time(nullptr);
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&t_now), "%Y%m%d_%H%M");
  const std::string run_ts = oss.str();

  const std::string urdf_filename = "../Robots/unitree-go2/go2_" + leg + ".urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  pinocchio::Data data(model);

  model.friction.setZero();
  model.damping.setZero();
  model.armature.setZero();

  // Initialize data
  bool include_offset_input = true;
  const std::string data_dir =
      "../Examples/Unitree_Go2/SystemIdentification/ParametersIdentification/"
      "full_params_data/friction/" + leg + "/physical/";

  const std::string posFile    = data_dir + "q_downsampled_filtered_"    + data_ts + ".csv";
  const std::string velFile    = data_dir + "q_d_downsampled_filtered_"  + data_ts + ".csv";
  const std::string accFile    = data_dir + "q_dd_downsampled_filtered_" + data_ts + ".csv";
  const std::string torqueFile = data_dir + "tau_downsampled_filtered_"  + data_ts + ".csv";

  Eigen::MatrixXd posData = Utils::initializeEigenMatrixFromFile(posFile);
  Eigen::MatrixXd velData = Utils::initializeEigenMatrixFromFile(velFile);
  Eigen::MatrixXd accData = Utils::initializeEigenMatrixFromFile(accFile);
  Eigen::MatrixXd torqueData = Utils::initializeEigenMatrixFromFile(torqueFile);

  int N = posData.rows(); // number of time steps
  std::shared_ptr<Eigen::MatrixXd> posDataPtr_ =
      std::make_shared<Eigen::MatrixXd>(posData.transpose());
  std::shared_ptr<Eigen::MatrixXd> velDataPtr_ =
      std::make_shared<Eigen::MatrixXd>(velData.transpose());
  std::shared_ptr<Eigen::MatrixXd> accDataPtr_ =
      std::make_shared<Eigen::MatrixXd>(accData.transpose());
  std::shared_ptr<Eigen::MatrixXd> torqueDataPtr_ =
      std::make_shared<Eigen::MatrixXd>(torqueData.transpose());

  // Initialize Kinova optimizer
  SmartPtr<FrictionParametersIdentification> mynlp =
      new FrictionParametersIdentification();
  try {
    mynlp->set_parameters(model, posDataPtr_, velDataPtr_, accDataPtr_,
                          torqueDataPtr_, include_offset_input);
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(
        "Error initializing Ipopt class! Check previous error message!");
  }

  SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetNumericValue("tol", 1e-4);
  app->Options()->SetNumericValue("constr_viol_tol", 1e-4);
  app->Options()->SetNumericValue("max_wall_time", 0.2);
  app->Options()->SetIntegerValue("print_level", 5);
  app->Options()->SetIntegerValue("max_iter", 50);
  app->Options()->SetStringValue("mu_strategy", "monotone");
  app->Options()->SetStringValue("linear_solver", "ma57");
  app->Options()->SetStringValue("ma57_automatic_scaling", "yes");
  if (mynlp->enable_hessian) {
    app->Options()->SetStringValue("hessian_approximation", "exact");
  } else {
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  }

  // // For gradient checking
  // app->Options()->SetStringValue("output_file", "ipopt.out");
  // app->Options()->SetStringValue("derivative_test", "second-order");
  // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-7);
  // app->Options()->SetNumericValue("derivative_test_tol", 1e-5);

  // Initialize the IpoptApplication and process the options
  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    throw std::runtime_error("Error during initialization of optimization!");
  }

  // Run ipopt to solve the optimization problem
  double solve_time = 0;
  try {
    auto start = std::chrono::high_resolution_clock::now();

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    auto end = std::chrono::high_resolution_clock::now();
    solve_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    std::cout << "Total solve time: " << solve_time << " milliseconds.\n";
    std::cout << "parameter solution: " << (mynlp->solution).transpose()
              << std::endl;

    // Write the friction parameters into the file
    std::ofstream solution(data_dir + "friction_parameters_solution_filtered_" + run_ts + ".csv");
    std::cout << "Solution saved: " << data_dir + "friction_parameters_solution_filtered_" + run_ts + ".csv\n";

    solution << std::setprecision(16);
    for (int i = 0; i < mynlp->solution.size(); i++) {
      solution << mynlp->solution(i) << std::endl;
    }

    // Write the estimate tau into the file
    // case 1: training data is the vaildation data
    // case 2: training data and vaildation data are different

    // Obtain the friction, damp and armature from the solution
    Eigen::VectorXd friction = mynlp->solution.head(model.nv); // 12 values
    Eigen::VectorXd damping = mynlp->solution.segment(model.nv, model.nv); // next 12 values
    Eigen::VectorXd armature = mynlp->solution.segment(2 * model.nv, model.nv); // next 12 values
    Eigen::VectorXd offset = Eigen::VectorXd::Zero(model.nv);
    if (include_offset_input) {
      offset = mynlp->solution.tail(model.nv); // last 12 values 
    }

    // case 1
    // Calculate the inertia tau
    Eigen::MatrixXd tau_inertials = Eigen::MatrixXd::Zero(model.nv, N); // rows: number of joints, cols: number of time steps
    
    for (int i = 0; i < N; i++) {
      const Eigen::VectorXd &q = posDataPtr_->col(i);
      const Eigen::VectorXd &v = velDataPtr_->col(i);
      const Eigen::VectorXd &a = accDataPtr_->col(i);

      pinocchio::rnea(model, data, q, v, a);
      tau_inertials.col(i) = data.tau;
    }

    std::ofstream estimate_tau(data_dir + "friction_estimate_tau_filtered_" + run_ts + ".csv");

    for (Index i = 0; i < N; i++) {
      const Eigen::VectorXd &q_d = velDataPtr_->col(i);
      const Eigen::VectorXd &q_dd = accDataPtr_->col(i);
      const Eigen::VectorXd &q = posDataPtr_->col(i);
      const Eigen::VectorXd &tau_inertial = tau_inertials.col(i);
      Eigen::VectorXd tau_estimated = Eigen::VectorXd::Zero(model.nv);

      Eigen::VectorXd total_friction_force =
          (friction.cwiseProduct(q_d.cwiseSign()) + damping.cwiseProduct(q_d) +
           armature.cwiseProduct(q_dd) + offset);
      tau_estimated = tau_inertial + total_friction_force;

      for (int j = 0; j < tau_estimated.size(); ++j) {
        estimate_tau << tau_estimated(j);
        if (j < tau_estimated.size() - 1) {
          estimate_tau << " ";
        }
      }
      estimate_tau << std::endl; // this should be the tau from the data. 
    }

    // case 2
    // using the vaildation set with the training parameters
    // const std::string posFile_check =
    // "../Examples/Kinova/SystemIdentification/ParametersIdentification/full_params_data/q_downsampled_5.csv";
    // const std::string velFile_check =
    // "../Examples/Kinova/SystemIdentification/ParametersIdentification/full_params_data/q_d_downsampled_5.csv";
    // const std::string accFile_check =
    // "../Examples/Kinova/SystemIdentification/ParametersIdentification/full_params_data/q_dd_downsampled_5.csv";

    // Eigen::MatrixXd posData_check =
    // Utils::initializeEigenMatrixFromFile(posFile_check); Eigen::MatrixXd
    // velData_check = Utils::initializeEigenMatrixFromFile(velFile_check);
    // Eigen::MatrixXd accData_check =
    // Utils::initializeEigenMatrixFromFile(accFile_check);

    // int N = posData_check.rows();

    // std::shared_ptr<Eigen::MatrixXd> posDataCheckPtr_ =
    // std::make_shared<Eigen::MatrixXd>(posData_check.transpose());
    // std::shared_ptr<Eigen::MatrixXd> velDataCheckPtr_ =
    // std::make_shared<Eigen::MatrixXd>(velData_check.transpose());
    // std::shared_ptr<Eigen::MatrixXd> accDataCheckPtr_ =
    // std::make_shared<Eigen::MatrixXd>(accData_check.transpose());

    // // compute inertia torque from data
    // Eigen::MatrixXd tau_inertials = Eigen::MatrixXd::Zero(model.nv, N);
    // for (int i = 0; i < N; i++) {
    //     const Eigen::VectorXd& q = posDataCheckPtr_ ->col(i);
    //     const Eigen::VectorXd& v = velDataCheckPtr_->col(i);
    //     const Eigen::VectorXd& a = accDataCheckPtr_->col(i);

    //     pinocchio::rnea(model, data, q, v, a);
    //     tau_inertials.col(i) = data.tau;
    // }

    // const std::string outputfolder1 =
    // "../Examples/Kinova/SystemIdentification/ParametersIdentification/full_params_data/";
    // std::ofstream estimate_tau(outputfolder1 + "friction_estimate_tau_"  +
    // std::string(argv[1]) +".csv");

    // for (Index i =0 ; i < N; i++) {
    //     const Eigen::VectorXd& q_d = velDataCheckPtr_->col(i);
    //     const Eigen::VectorXd& q_dd = accDataCheckPtr_->col(i);
    //     const Eigen::VectorXd& q = posDataCheckPtr_->col(i);
    //     const Eigen::VectorXd& tau_inertial = tau_inertials.col(i);
    //     Eigen::VectorXd tau_estimated = Eigen::VectorXd::Zero(model.nv);

    //     Eigen::VectorXd total_friction_force =
    //         (friction.cwiseProduct(q_d.cwiseSign()) +
    //         damping.cwiseProduct(q_d) +
    //         armature.cwiseProduct(q_dd) +
    //         offset);
    //     tau_estimated = tau_inertial + total_friction_force;

    //     for (int j = 0; j < tau_estimated.size(); ++j) {
    //         estimate_tau << tau_estimated(j);
    //         if (j < tau_estimated.size() - 1) {
    //             estimate_tau << " ";
    //         }
    //     }
    //     estimate_tau << std::endl;
    // }
  } catch (std::exception &e) {
    throw std::runtime_error(
        "Error solving optimization problem! Check previous error message!");
  }

  return 0;
}
