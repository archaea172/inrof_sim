#include "mppi_swerve.hpp"

/*mppi begin*/
/*model func begin*/
Eigen::VectorXd MppiSwerve::model(const Eigen::VectorXd &input, const Eigen::VectorXd &pre_state)
{
    Eigen::VectorXd post_state(this->input_dim_);
    post_state = pre_state + this->control_cycle_*input;
    return post_state;
}

Eigen::MatrixXd MppiSwerve::generate_model_state(const Eigen::MatrixXd &input_array, const Eigen::VectorXd &init_state)
{
    Eigen::MatrixXd state_array(this->predict_horizon_, this->input_dim_);
    state_array.row(0) = init_state;
    for (size_t i = 1; i < (size_t)predict_horizon_; i++) state_array.row(i) = this->model(input_array.row(i), state_array.row(i-1)).transpose();
    return state_array;
}
/*model func end*/

/*swerve drive cal begin*/
std::vector<std::vector<double>> MppiSwerve::swerve_cal(const double Theta, const std::vector<double> &V)
{
    std::vector<std::vector<double>> velocity(3, std::vector<double>(2, 0));
    std::vector<std::vector<double>> wheelandstare(2, std::vector<double>(3, 0));

    const float a[3] = {M_PI/6, 5*M_PI/6, 3*M_PI/2};

    for (int i = 0; i < 3; i++){
        velocity[i][0] = V[0] - V[2]*this->R*sin(Theta + a[i]);
        velocity[i][1] = V[1] + V[2]*this->R*cos(Theta + a[i]);
    }

    wheelandstare[0][0] = sqrt(pow(velocity[0][0], 2.0) + pow(velocity[0][1], 2.0))/r;
    wheelandstare[0][1] = sqrt(pow(velocity[1][0], 2.0) + pow(velocity[1][1], 2.0))/r;
    wheelandstare[0][2] = sqrt(pow(velocity[2][0], 2.0) + pow(velocity[2][1], 2.0))/r;


    wheelandstare[1][0] = atan2(velocity[0][1], velocity[0][0]) - Theta;
    wheelandstare[1][1] = atan2(velocity[1][1], velocity[1][0]) - Theta;
    wheelandstare[1][2] = atan2(velocity[2][1], velocity[2][0]) - Theta;

    return wheelandstare;
}
std::vector<std::vector<double>> MppiSwerve::swerve_control(const std::vector<std::vector<double>> &swerve_num)
{
    std::vector<std::vector<double>> post_swerve = swerve_num;
    for (int i = 0; i < 3; i++) {
        if (post_swerve[1][i] < 0) {
            post_swerve[1][i] += M_PI;
            post_swerve[0][i] *= -1;
        }
    }
    if (0 == post_swerve[0][0]) post_swerve[1][0] = M_PI/6;
    if (0 == post_swerve[0][1]) post_swerve[1][1] = M_PI/6*5;
    if (0 == post_swerve[0][2]) post_swerve[1][2] = M_PI/2;

    return post_swerve;
}
/*swerve drive cal end*/

/*evaluation begin*/
double MppiSwerve::calc_evaluation(Eigen::MatrixXd InputList, Eigen::MatrixXd StateList, Eigen::MatrixXd GoalPose)
{
    Eigen::VectorXd S_array(this->num_evaluation);
    S_array[0] = this->evaluate_ref();
    double S = gain_vector.dot(S_array);
    return S;
}
/*evaluation end*/
/*mppi end*/