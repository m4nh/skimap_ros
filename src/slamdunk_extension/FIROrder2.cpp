#include "FIROrder2.h"
#include <kdl/frames.hpp>

FIROrder2::FIROrder2(int data_size, double cutoff, double quality) {
    this->data_size = data_size;
    this->Q = quality;
    this->updateParameters(cutoff);
    this->ready = false;

    this->data.resize(data_size);
    this->output.resize(data_size);
    for (int i = 0; i < data_size; i++) {
        this->data[i].resize(4);
    }
}

FIROrder2::~FIROrder2() {

}

void FIROrder2::setSampleTime(double sample_time) {
    this->Fs = 1.0 / sample_time;
    this->updateParameters(this->Fc);
}

void FIROrder2::updateParameters(double cutoff) {
    this->Fc = cutoff;

    this->W = tan(M_PI * this->Fc / this->Fs);
    this->N = 1.0 / (pow(this->W, 2) + this->W / Q + 1);
    this->B0 = this->N * pow(this->W, 2);
    this->B1 = 2 * this->B0;
    this->B2 = this->B0;
    this->A1 = 2 * this->N * (pow(this->W, 2) - 1);
    this->A2 = this->N * (pow(this->W, 2) - this->W / this->Q + 1);
}

void FIROrder2::setInput(int i, double X) {

    double Acc =
            X * this->B0
            + this->data[i][0] * this->B1
            + this->data[i][1] * this->B2
            - this->data[i][2] * this->A1
            - this->data[i][3] * this->A2;

    this->data[i][3] = this->data[i][2];
    this->data[i][2] = Acc;
    this->data[i][1] = this->data[i][0];
    this->data[i][0] = X;

    //    printf("Set input %i, %f %f %f %f %f \n", i, this->B0,this->A1, this->A2, this->B1, this->B2);
    this->output[i] = Acc;
}

void FIROrder2::setInputs(double* Xs) {
    for (int i = 0; i < this->data_size; i++) {
        this->setInput(i, Xs[i]);

    }
}

void FIROrder2::setInitialData(double* Xs) {
    for (int i = 0; i < this->data_size; i++) {
        this->data[i][0] = Xs[i];
        this->data[i][1] = Xs[i];
        this->data[i][2] = Xs[i];
        this->data[i][3] = Xs[i];
        this->output[i] = Xs[i];
        //        printf("In data %i , %f\n", i, output[i]);
    }
    this->ready = true;
}

void FIROrder2::setInitialData(Eigen::Isometry3d& iso) {

    double* data;
    conversion(iso, data);
    setInitialData(data);
}

void FIROrder2::setInput(Eigen::Isometry3d& iso) {
    double* data;
    conversion(iso, data);
    setInputs(data);
}

void FIROrder2::conversion(Eigen::Isometry3d& iso, double*& data, bool reverse) {
    //    if (!reverse) {
    //        tf::Transform tf;
    //        tf::transformEigenToTF(iso, tf);
    //        KDL::Frame frame;
    //        tf::transformTFToKDL(tf, frame);
    //
    //        data = new double[7];
    //        data[0] = frame.p.x();
    //        data[1] = frame.p.y();
    //        data[2] = frame.p.z();
    //        frame.M.GetQuaternion(
    //                data[3],
    //                data[4],
    //                data[5],
    //                data[6]
    //                );
    //
    //    } else {
    //        KDL::Frame frame;
    //        frame.p = KDL::Vector(data[0], data[1], data[2]);
    //        frame.M = KDL::Rotation::Quaternion(data[3],
    //                data[4],
    //                data[5],
    //                data[6]
    //                );
    //        tf::Transform tf;
    //        tf::transformKDLToTF(frame, tf);
    //        tf::transformTFToEigen(tf, iso);
    //
    //    }
    if (!reverse) {
        tf::Transform tf;
        tf::transformEigenToTF(iso, tf);
        KDL::Frame frame;
        tf::transformTFToKDL(tf, frame);

        data = new double[6];
        data[0] = frame.p.x();
        data[1] = frame.p.y();
        data[2] = frame.p.z();
        frame.M.GetRPY(data[3], data[4], data[5]);

    } else {
        KDL::Frame frame;
        frame.p = KDL::Vector(data[0], data[1], data[2]);
        frame.M = KDL::Rotation::RPY(data[3], data[4], data[5]);
        tf::Transform tf;
        tf::transformKDLToTF(frame, tf);
        tf::transformTFToEigen(tf, iso);

    }
}

void FIROrder2::getOutput(Eigen::Isometry3d& iso) {
    double* data = &output[0];
    //    for (int i = 0; i < 7; i++) {
    //        printf("Out data %i , %f\n", i, data[i]);
    //    }

    conversion(iso, data, true);
}

