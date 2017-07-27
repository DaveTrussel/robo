#pragma once

#include "kinematics.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace std::chrono;


namespace robo{

// helper functions
#define now() high_resolution_clock::now()

VectorXd rand_joint_vector(int size, double min, double max){
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> distri(min, max);
    VectorXd vec(size);
    for(int i=0; i<size; ++i){
        vec[i] = distri(gen);
    }
    return vec;
}

void print_timing_result(vector<double> timings){
    double sum = 0;
    double median = 0;
    size_t len = timings.size();
    sort(timings.begin(), timings.end());
    double max = timings.back();
    double min = timings.front();
    for (double &dt: timings) {
        sum += dt;
    }
    double average = sum / len;
    if(len % 2 == 0){
        median = (timings[len/2] + timings[len/2-1]) / 2.0;
    }
    else{
        median = timings[len/2];
    }
    cout << "Timing results (in microseconds) from " << len << " runs:"<< endl
         << "Average: " << average << endl
         << "Median: " << median << endl
         << "Min: " << min << endl
         << "Max: " << max << endl;
}

void print_success_rates_IK(vector<Error_type> errors){
    int nr_no_error = count(errors.begin(), errors.end(), Error_type::no_error);
    double successrate = 100.0 * double(nr_no_error)/errors.size();
    cout << "Successrate: " << successrate << "%" << endl;
}

void print_joint_limits_compliance(vector<bool> joint_limit_compliances){
    int nr_in_JL = count(joint_limit_compliances.begin(), joint_limit_compliances.end(), true);
    double rate_in_JL = 100.0 * double(nr_in_JL)/joint_limit_compliances.size();
    cout << "Within joint limits: " << rate_in_JL << "%" << endl;
}

}