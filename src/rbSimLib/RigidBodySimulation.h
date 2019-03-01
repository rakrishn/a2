#pragma once

#include "KinematicEnergy.h"
#include <NewtonFunctionMinimizer.h>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2d)

using Eigen::Matrix;

struct TrackRBPoint {
    int rbIdx = -1;
    Vector2d local;
};

class RigidBodySimulation
{
public:
    void run() {
        newton.minimize(&energy, x);
    }

    void reset() {
        x = x0;
    }

    Matrix<double, -1, 2> recordTrajectory() const {
        assert(motorIdx != -1);

        auto &motor = fixedAngle()[motorIdx];
        const auto &rb = rigidbodies()[trackRBPoint.rbIdx];

        energy.fixedAngleEnabled = true;

        const double angleSave = motor.angle;

        const int nSteps = 100;
        Matrix<double, -1, 2> path(nSteps, 2);

        VectorXd x = x0;
        for (int i = 0; i < nSteps; ++i) {
            motor.angle = (double)i/(double)(nSteps-1) * 2*M_PI;
            newton.minimize(&energy, x);

            Vector2d p = rb.pWorld(x, trackRBPoint.local);
            path.block<1, 2>(i, 0) = p.transpose();
        }

        motor.angle = angleSave;

        return path;
    }

    void scaleRigidBody(int rbIdx, double scale) {
        assert(rbIdx >= 0 && rbIdx < energy.rigidbodies.size());

        auto &rb = energy.rigidbodies[rbIdx];
        rb.length *= scale;

        for(auto &joint : hingeJoints()){
            for (int j = 0; j < 2; ++j) {
                if(joint.rbIdx[j] == rbIdx){
                    joint.local[j][0] *= scale;
                }
            }
        }
        if(trackRBPoint.rbIdx == rbIdx)
            trackRBPoint.local *= scale;
    }

    void setRigidBodyLength(int rbIdx, double length){
        assert(rbIdx >= 0 && rbIdx < energy.rigidbodies.size());

        auto &rb = energy.rigidbodies[rbIdx];
        rb.length = length;

        for(auto &joint : hingeJoints()){
            for (int j = 0; j < 2; ++j) {
                if(joint.rbIdx[j] == rbIdx){
                    if(joint.local[j][0] != 0){
                        double s = (joint.local[j][0] < 0) ? -1 : 1;
                        joint.local[j][0] = s * length / 2;
                    }
                }
            }
        }

        if(trackRBPoint.rbIdx == rbIdx){
            double s = (trackRBPoint.local[0] < 0) ? -1 : 1;
            trackRBPoint.local[0] = s * length / 2;
        }
    }

    void setDesignParameters(const VectorXd &p) {
        assert(p.size() == rigidbodies().size());

        for (int i = 0; i < p.size(); i++){
            setRigidBodyLength(i, p[i]);
        }
    }

    VectorXd getDesignParameters() const {
        VectorXd p(rigidbodies().size());
        int i = 0;
        for (auto &rb : rigidbodies())
            p[i++] = rb.length;
        return p;
    }

    // for easier access
    const std::vector<RigidBody> &rigidbodies() const { return energy.rigidbodies;	}
    std::vector<RigidBody> &rigidbodies() { return energy.rigidbodies;	}
    const std::vector<HingeJoint> &hingeJoints() const { return energy.hingeJoints;	}
    std::vector<HingeJoint> &hingeJoints() { return energy.hingeJoints;	}
    const std::vector<FixedJoint> &fixed() const { return energy.fixed;	}
    std::vector<FixedJoint> &fixed() { return energy.fixed; }
    const std::vector<FixedAngleJoint> &fixedAngle() const { return energy.fixedAngle; }
    std::vector<FixedAngleJoint> &fixedAngle() { return energy.fixedAngle; }

public:
    NewtonFunctionMinimizer newton;
    KinematicEnergy energy;
    VectorXd x, x0;
    TrackRBPoint trackRBPoint;
    int motorIdx = -1;
};
