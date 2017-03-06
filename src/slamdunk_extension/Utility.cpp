/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Utility.cpp
 * Author: Duo Gao
 * 
 * Created on 13 luglio 2016, 15.35
 */

#include <slamdunk_extension/Utility.h>

Utility::Utility() {
}

Utility::Utility(const Utility& orig) {
}

Utility::~Utility() {
}

/**
 * KDL Frame to Eigen Matrix 4x4
 * @param frame KDL Frame
 * @param mat Eigen Matrix 4x4
 */
void Utility::kdl_to_eigen_4x4_d(KDL::Frame& frame, Eigen::Matrix4d& mat) {
        for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                        mat(i, j) = frame.M(i, j);
                }
        }

        mat(0, 3) = frame.p[0];
        mat(1, 3) = frame.p[1];
        mat(2, 3) = frame.p[2];
        mat(3, 3) = 1;
}

/**
 * Creates KDL Frame
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 * @param out_frame OUTPUT
 */
void Utility::create_kdl_frame(float x, float y, float z, float roll, float pitch, float yaw, KDL::Frame& out_frame) {

        out_frame.M = KDL::Rotation::RPY(
                roll,
                pitch,
                yaw
                );

        out_frame.p[0] = x;
        out_frame.p[1] = y;
        out_frame.p[2] = z;
}

/**
 * Creates Eigen4x4 Matrix
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 * @param mat
 */

void Utility::create_eigen_4x4_d(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4d& mat) {
        mat = Eigen::Matrix4d::Identity();
        KDL::Frame frame;
        create_kdl_frame(x, y, z, roll, pitch, yaw, frame);
        kdl_to_eigen_4x4_d(frame, mat);
}

/**
 * Builds TF from geometry_msgs::Pose TODO: reverse
 */

void Utility::eigen_4x4_to_geometrypose_d(Eigen::Matrix4d& mat,geometry_msgs::Pose& pose){

        pose.position.x = mat(0,3);
        pose.position.y = mat(1,3);
        pose.position.z = mat(2,3);

        tf::Matrix3x3 m;
        for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                        m[i][j]=mat(i,j);

        tf::Quaternion q;
        m.getRotation(q);
        q.normalize();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
}

/**
 * Builds TF from geometry_msgs::Pose TODO: reverse
 */
void Utility::eigen_4x4_d_to_tf(Eigen::Matrix4d& t,  tf::Transform& tf, bool reverse = false){
        if(!reverse) {
                tf.setOrigin( tf::Vector3(
                                      t(0,3),
                                      t(1,3),
                                      t(2,3)
                                      ));

                tf::Matrix3x3 rot;
                for(int i = 0; i < 3; i++)
                        for(int j = 0; j < 3; j++)
                                rot[i][j] = t(i,j);

                tf::Quaternion q;
                rot.getRotation(q);
                q.normalize();
                tf.setRotation(q);
        }else{
                t = Eigen::Matrix4d::Identity();
                t(0,3) = tf.getOrigin()[0];
                t(1,3) = tf.getOrigin()[1];
                t(2,3) = tf.getOrigin()[2];

                tf::Matrix3x3 rot;
                rot.setRotation(tf.getRotation());
                for(int i = 0; i < 3; i++)
                        for(int j = 0; j < 3; j++)
                                t(i,j) = rot[i][j];

        }
}




void Utility::HSVtoRGB(double &r, double &g, double &b, double h, double s, double v) {
    int i;
    double f, p, q, t;

    if (s == 0) {
        // achromatic (grey)
        r = g = b = v;
        return;
    }

    h /= 60; // sector 0 to 5
    i = floor(h);
    f = h - i; // factorial part of h
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));

    switch (i) {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        default: // case 5:
            r = v;
            g = p;
            b = q;
            break;
    }

}