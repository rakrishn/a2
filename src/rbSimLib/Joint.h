#pragma once

#include "RigidBody.h"

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class HingeJoint {
public:

    HingeJoint(int rb0Idx, int rb1Idx, Vector2d local0, Vector2d local1)
        : rbIdx({rb0Idx, rb1Idx}), local({local0, local1}) {
    }

    VectorXd constraints(const VectorXd &x, const RigidBody &rb0, const RigidBody &rb1) const {
        Vector2d p0 = rb0.pWorld(x, local[0]);
        Vector2d p1 = rb1.pWorld(x, local[1]);
        return (p1-p0);
    }

    Matrix<double, 2, 6> jacobian(const VectorXd x, const RigidBody &rb0, const RigidBody &rb1) const {
        Matrix<double, 2, 6> dCdx = Matrix<double, 2, 6>::Zero();

        //////////////////////////
        // your code for Task 1.2
        //////////////////////////

        return dCdx;
    }

public:
    std::array<int, 2> rbIdx;
    std::array<Vector2d, 2> local;
};

class FixedJoint
{
public:
    VectorXd constraints(const VectorXd &x, const RigidBody &rb) const {
        return (pos - rb.pWorld(x, localPos));
    }

    Matrix<double, 2, 3> jacobian(const VectorXd x, const RigidBody &rb) const {
        Matrix<double, 2, 3> dCdx = Matrix<double, 2, 3>::Zero();

        //////////////////////////
        // your code for Task 1.2
        //////////////////////////

        return dCdx;
    }

public:
    Vector2d pos;       // world coordinates
    int rbIdx;          // index of rigid body
    Vector2d localPos;  // position in rigid body coordinates
};

class FixedAngleJoint
{
public:
    VectorXd constraints(const VectorXd &x, const RigidBody &rb) const {
        VectorXd c(1);
        c << (rb.theta(x) - angle);
        return c;
    }

    Matrix<double, 1, 3> jacobian(const VectorXd x, const RigidBody &rb) const {
        Matrix<double, 1, 3> dCdx = Matrix<double, 1, 3>::Zero();

        //////////////////////////
        // your code for Task 1.2
        //////////////////////////

        return dCdx;
    }

public:
    int rbIdx;
    mutable double angle;
};
