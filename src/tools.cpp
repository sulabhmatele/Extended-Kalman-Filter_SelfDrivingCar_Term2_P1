#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() = default;

Tools::~Tools() = default;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
    {
        VectorXd rmse(4);
        rmse << 0,0,0,0;

        // Checking the validity of the following inputs:
        //  * the estimation vector size should not be zero
        //  * the estimation vector size should equal ground truth vector size

        if(estimations.size() != ground_truth.size()
           || estimations.empty())
        {
            cout << "Invalid estimation or ground_truth data" << endl;
            return rmse;
        }

        //accumulate squared residuals
        for(unsigned int i=0; i < estimations.size(); ++i)
        {

            VectorXd residual = estimations[i] - ground_truth[i];

            //coefficient-wise multiplication
            residual = residual.array()*residual.array();
            rmse += residual;
        }

            rmse = rmse/ estimations.size();

            rmse = rmse.array().sqrt();

            //return the result
            return rmse;
    }


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
    {
        MatrixXd Hj = MatrixXd(3,4);

        Hj << 0,0,0,0,
              0,0,0,0,
              0,0,0,0;

        //recover state parameters
        float px = x_state(0);
        float py = x_state(1);
        float vx = x_state(2);
        float vy = x_state(3);

        float deno = px * px + py * py;
        float fact_0 = sqrt(deno);
        float fact_1 = vx*py - vy*px;
        float fact_2 = vy*px - vx*py;

        if(fabs(deno) > 0.0001)
        {
            Hj << px / fact_0, py / fact_0, 0, 0,
                  -py / deno, px / deno, 0, 0,
                  py * (fact_1) / pow(deno, 1.5), px * (fact_2) / pow(deno, 1.5), px / fact_0, py / fact_0;
        }

        return Hj;
    }

