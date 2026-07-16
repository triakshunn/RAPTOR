#include "EndEffectorParametersIdentificationMomentum.h"
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace RAPTOR;

const std::string folder_name =
    "../Examples/Unitree_Go2/SystemIdentification/ParametersIdentification/"
    "full_params_data/";

const int H = 5;
const int downsample_rate = 1;

int main(int argc, char *argv[]) {
  // check if the file number is provided
  if (argc < 4) {
        throw std::invalid_argument(
            "Usage: ./Go2_SysidInertialMomentum_test <leg> <data_ts> <friction_ts>\n"
            "  e.g. ./Go2_SysidInertialMomentum_test FR 20250707_1430 20250707_1432");
    }
    
  const std::string leg         = std::string(argv[1]);
  const std::string data_ts     = std::string(argv[2]);
  const std::string friction_ts = std::string(argv[3]);

  std::time_t t_now = std::time(nullptr);
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&t_now), "%Y%m%d_%H%M");
  const std::string run_ts = oss.str();
 {
  
    // Load the robot model

    std::cout << "\n=== Processing leg: " << leg << " ===\n";

    pinocchio::Model model;
    pinocchio::urdf::buildModel("../Robots/unitree-go2/go2_" + leg + ".urdf", model);

    // const std::string friction_file =
    //         folder_name + "friction/" + leg + "/physical/friction_parameters_solution_filtered_20260715_2122.csv"; // change this to lower line
    const std::string friction_file =
             folder_name + "friction/" + leg + "/physical/friction_parameters_solution_filtered_" + friction_ts + ".csv";
    Eigen::VectorXd fp =
          Utils::initializeEigenMatrixFromFile(friction_file).col(0);
    if (fp.size() != 3 * model.nv && fp.size() != 4 * model.nv)
        throw std::runtime_error("Friction file size mismatch for leg: " + leg);
    model.friction = fp.head(model.nv);
    model.damping  = fp.segment(model.nv, model.nv);
    model.armature = fp.segment(2 * model.nv, model.nv);

    Eigen::VectorXd offset = Eigen::VectorXd::Zero(model.nv);
    if (fp.size() == 4 * model.nv)
        offset = fp.tail(model.nv);

    // load the data
    const std::string data_dir = folder_name + "inertial/" + leg + "/physical/";

    // Sensor noise info
    SensorNoiseInfo sensor_noise(model.nv);
    // sensor_noise.position_error << Utils::deg2rad(0.02),
    //                                Utils::deg2rad(0.02),
    //                                Utils::deg2rad(0.02),
    //                                Utils::deg2rad(0.02),
    //                                Utils::deg2rad(0.011),
    //                                Utils::deg2rad(0.011),
    //                                Utils::deg2rad(0.011); // from Kinova
    //                                official support, resolution of joint
    //                                encoders
    // sensor_noise.velocity_error = 5 * sensor_noise.position_error;
    sensor_noise.position_error.setZero();
    sensor_noise.velocity_error.setZero();
    sensor_noise.acceleration_error_type =
        SensorNoiseInfo::SensorNoiseType::Ratio;
    sensor_noise.acceleration_error.setConstant(0.1); //  (this is the sensor noise added to "torque" not accelaration data, nomenclature )(also this is defined for all joints, when each joint might have different unceratinity )


    // ToDo: Find the sensor noise for each joint sensor for torque by keeping it at rest and recording torque values.
    // Initialize the Ipopt problem
    SmartPtr<EndEffectorParametersIdentificationMomentum> mynlp =
        new EndEffectorParametersIdentificationMomentum();

  
    double setup_time = 0;
    try {
      auto start = std::chrono::high_resolution_clock::now();
      mynlp->set_parameters(model, offset, 6e-7); // init params for nlp
      mynlp->add_trajectory_file( data_dir + "traj_data_filtered_" + data_ts + ".csv", sensor_noise, H,
                                 TimeFormat::Second, downsample_rate);
      auto end = std::chrono::high_resolution_clock::now();
      setup_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
              .count();
      std::cout << "Setup time: " << setup_time << " milliseconds.\n";
    } catch (std::exception &e) {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error(
          "Error initializing Ipopt class! Check previous error message!");
    }


    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-10);
    app->Options()->SetNumericValue("max_wall_time", 30.0); // changed from 10>30
    app->Options()->SetIntegerValue("print_level", 5);
    app->Options()->SetIntegerValue("max_iter", 200); // changed from 100-> 200
    app->Options()->SetStringValue("mu_strategy", "adaptive");
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
    // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-8);
    // app->Options()->SetNumericValue("derivative_test_tol", 1e-5);
    // app->Options()->SetNumericValue("point_perturbation_radius", 1);

     // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
      throw std::runtime_error("Error during initialization of optimization!");
    }

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
    } catch (std::exception &e) {
      throw std::runtime_error(
          "Error solving optimization problem! Check previous error message!");
    }

    std::cout << "parameter solution: " << mynlp->theta_solution.transpose() << "\n"; // removed unparametrized solutions solution variable
    std::cout << "uncertainty:        " << mynlp->theta_uncertainty.transpose() << "\n";
    std::cout << "groundtruth:        " << mynlp->phi_original.transpose() << "\n";

    const std::string out_path = data_dir + "inertial_parameters_solution_filtered_" + run_ts + ".csv";
    std::ofstream out(out_path);
    for (int i = 0; i < mynlp->theta_solution.size(); i++)
       out << mynlp->theta_solution(i) << (i < mynlp->theta_solution.size() - 1 ? "," : "\n");
    std::cout << "Results saved to: " << out_path << "\n";
            }

  return 0;
}

// 1655: 6e-8, 1657: 2e-8, 1658: 2e-7, 1700: 6e-7
