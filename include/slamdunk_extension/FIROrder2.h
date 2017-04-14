/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <utility>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>

#ifndef FIRORDER2_H
#define FIRORDER2_H

class FIROrder2
{
  public:
    FIROrder2(int data_size, double cutoff, double quality = 0.5);
    virtual ~FIROrder2();
    void updateParameters(double cutoff);
    void setSampleTime(double sample_time);

    void setInput(int index, double X);
    void setInputs(double *Xs);
    void setInput(Eigen::Isometry3d &iso);
    void setInitialData(double *Xs);
    void setInitialData(Eigen::Isometry3d &iso);

    void getOutput(Eigen::Isometry3d &iso);

    double Fc;
    double Fs;
    double Q;
    double W;
    double N;
    double B0;
    double B1;
    double B2;
    double A1;
    double A2;

    std::vector<double> output;
    std::vector<std::vector<double>> data;
    bool ready;

  private:
    void conversion(Eigen::Isometry3d &iso, double *&data, bool reverse = false);
    int data_size;
};

#endif /* FIRORDER2_H */