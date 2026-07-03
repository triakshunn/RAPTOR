#include "EndEffectorParametersIdentification.h"
#include <fstream>


using namespace RAPTOR;

const std::string folder_name =
    "../Examples/Unitree_Go1/SystemIdentification/ParametersIdentification/"
    "full_params_data/";



int main(int argc, char* argv[]) {


  

    if (argc < 2) {
         throw std::invalid_argument(
             "Usage: ./Go1_SysidInertial_test FR [FL RR RL ...]");
     }
    

     for (int leg_idx = 1; leg_idx < argc; leg_idx++) {

        const std::string leg = std::string(argv[leg_idx]);
        std::cout << "\n=== Processing leg: " << leg << " ===\n";

        pinocchio::Model model;
        pinocchio::urdf::buildModel(
             "../Robots/unitree-go1/go1_" + leg + ".urdf", model);
        pinocchio::Data data(model);

        const std::string friction_file =
            folder_name + "friction/" + leg + "/friction_parameters_solution.csv";
        
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
    
        // // Generate a random trajectory data (Not implementing this for the Go1 sysID)
        // const double T = 10.0; // duration of the trajectory
        // const int N = 1000; // number of samples in the data
    
        //     // to compute the acceleration and then the torque
        // std::shared_ptr<BezierCurves> test_trajectory = std::make_shared<BezierCurves>(
        //     T, N, model.nv, TimeDiscretization::Uniform, 3);
        // test_trajectory->compute(Eigen::VectorXd::Random(test_trajectory->varLength), false); // not used by us??? 
    
        //     // to store the position, velocity and torque data for system identification
        // std::shared_ptr<TrajectoryData> trajectory_data = std::make_shared<TrajectoryData>(
        //     T, N, model.nv);
    
        //     // the matrix that stores the acceleration data
        // Eigen::MatrixXd acceleration(N, model.nv);
    
        // for (int i = 0; i < N; i++) {
        //     acceleration.row(i) = test_trajectory->q_dd(i);
    
        //     pinocchio::rnea(model, data, test_trajectory->q(i), test_trajectory->q_d(i), test_trajectory->q_dd(i));
    
        //     trajectory_data->q(i) = test_trajectory->q(i);
        //     trajectory_data->q_d(i) = test_trajectory->q_d(i);
        //     trajectory_data->q_dd(i) = data.tau + 
        //                                model.friction.cwiseProduct(test_trajectory->q_d(i).cwiseSign()) +
        //                                model.damping.cwiseProduct(test_trajectory->q_d(i)) +
        //                                offset;
        // }
      
        
        const std::string data_dir = folder_name + "inertial/" + leg + "/";
        
        
        // Initialize the Ipopt problem
        SmartPtr<EndEffectorParametersIdentification> mynlp = new EndEffectorParametersIdentification();
    
        double setup_time = 0;
        try {
            auto start = std::chrono::high_resolution_clock::now();
            mynlp->set_parameters(model, offset, 1e-6); 
            mynlp->add_trajectory_file(
                data_dir + "traj_data.csv",
                data_dir + "acceleration.csv",
                 TimeFormat::Second);
            auto end = std::chrono::high_resolution_clock::now();
            setup_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "Setup time: " << setup_time << " milliseconds.\n";
         }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
            throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
        }
    
        SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    
        app->Options()->SetNumericValue("tol", 1e-10);
        app->Options()->SetNumericValue("max_wall_time", 30.0);  // changed from 1.0 to 30.0
        app->Options()->SetIntegerValue("print_level", 5);
        app->Options()->SetIntegerValue("max_iter", 200);  // changed from 100 to 200
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetStringValue("linear_solver", "ma57");
        app->Options()->SetStringValue("ma57_automatic_scaling", "yes");
        if (mynlp->enable_hessian) {
            app->Options()->SetStringValue("hessian_approximation", "exact");
        }
        else {
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
        if( status != Solve_Succeeded ) {
            throw std::runtime_error("Error during initialization of optimization!");
        }
    
        // Run ipopt to solve the optimization problem
        double solve_time = 0;
        try {
            auto start = std::chrono::high_resolution_clock::now();
            // Ask Ipopt to solve the problem
            status = app->OptimizeTNLP(mynlp);
    
            auto end = std::chrono::high_resolution_clock::now();
            solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "Total solve time: " << solve_time << " milliseconds.\n";
        }
        catch (std::exception& e) {
            throw std::runtime_error("Error solving optimization problem! Check previous error message!");
        }
    
        std::cout << "parameter solution: " << mynlp->theta_solution.transpose() << "\n";
        std::cout << "groundtruth:        " << mynlp->phi_original.transpose() << "\n";

        const std::string out_path = data_dir + "inertial_parameters_solution.csv";
        std::ofstream out(out_path);
        for (int i = 0; i < mynlp->theta_solution.size(); i++)
             out << mynlp->theta_solution(i) << "\n";
        std::cout << "Results saved to: " << out_path << "\n";
     }

     return 0;
    }

