#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
    {
        VectorXd rmse(4);
        rmse << 0,0,0,0;

        // Checking the validity of the following inputs:
        //  * the estimation vector size should not be zero
        //  * the estimation vector size should equal ground truth vector size

        if((!estimations.empty()) && (estimations.size() == ground_truth.size()))
        {
            VectorXd residual;

            //accumulate squared residuals
            for (int i = 0; i < estimations.size(); ++i)
            {
                residual = (estimations[i] - ground_truth[i]);
                residual = residual.array() * residual.array();
                rmse += residual;
            }

            rmse = rmse/ estimations.size();

            rmse = rmse.array().sqrt();

            //return the result
            return rmse;
        }
        return rmse;
    }


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
    {
        MatrixXd Hj(3,4);
        //recover state parameters
        const double & px = x_state(0);
        const double & py = x_state(1);
        const double & vx = x_state(2);
        const double & vy = x_state(3);

        double deno = px * px + py * py;
        double fact_1 = vx*py - vy*px;
        double fact_2 = vy*px - vx*py;

        Hj(0,0) = (sqrt(deno) != 0) ? px/sqrt(deno): 0;
        Hj(0,1) = (sqrt(deno) != 0) ? py/sqrt(deno): 0;
        Hj(0,2) = 0;
        Hj(0,3) = 0;

        Hj(1,0) = (deno != 0) ? -py/deno: 0;
        Hj(1,1) = (deno != 0) ? px/deno: 0;
        Hj(1,2) = 0;
        Hj(1,3) = 0;

        Hj(2,0) = (pow(deno, 3/2)) != 0.0 ? py *(fact_1) /pow(deno, 3/2) : 0;
        Hj(2,1) = (pow(deno, 3/2)) != 0.0 ? px *(fact_2) /pow(deno, 3/2) : 0;
        Hj(2,2) = (sqrt(deno) != 0) ? px/sqrt(deno): 0;
        Hj(2,3) = (sqrt(deno) != 0) ? py/sqrt(deno): 0;

        return Hj;
    }

