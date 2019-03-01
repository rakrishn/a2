#pragma once

#include "RigidBody.h"
#include "Joint.h"

#include <vector>

#include <ObjectiveFunction.h>

class KinematicEnergy : public ObjectiveFunction
{
public:
    virtual double evaluate(const VectorXd& x) const {

        VectorXd c = constraints(x);
        return 0.5 * c.dot(c);
    }

    virtual void addGradientTo(const VectorXd& x, VectorXd& grad) const {
        grad += jacobian(x).transpose() * constraints(x);
    }

    virtual void addHessianEntriesTo(const VectorXd& x, std::vector<Triplet<double>>& hessianEntries) const {
        MatrixXd hess = MatrixXd::Zero(x.size(), x.size());
        hess = jacobian(x).transpose() * jacobian(x);

        for (int i = 0; i < hess.rows(); ++i) {
            for (int j = 0; j < hess.cols(); ++j) {
                hessianEntries.push_back({i, j, hess(i, j)});
            }
        }
    }

    VectorXd constraints(const VectorXd &x) const {
        VectorXd c(getNumConstraints());

        int i = 0;
        for (const auto &j : hingeJoints) {
            c.segment<2>(i) = j.constraints(x, rigidbodies[j.rbIdx[0]], rigidbodies[j.rbIdx[1]]);
            i += 2;
        }

        for(const auto &f : fixed) {
            c.segment<2>(i) = f.constraints(x, rigidbodies[f.rbIdx]);
            i += 2;
        }

        if(fixedAngleEnabled)
            for(const auto &f : fixedAngle) {
                c.segment<1>(i) = f.constraints(x, rigidbodies[f.rbIdx]);
                i += 1;
            }

        return c;
    }

    MatrixXd jacobian(const VectorXd &x) const {
        MatrixXd dCdx(getNumConstraints(), x.size());
        dCdx.setZero();

        int i = 0; // keep track of how many constraints were processed
        for (const auto &j : hingeJoints) {
            // get the rigid bodies that are connected to the hinge joint
            const RigidBody &rb0 = rigidbodies[j.rbIdx[0]];
            const RigidBody &rb1 = rigidbodies[j.rbIdx[1]];

            //////////////////////////
            // your code for Task 1.3
            //////////////////////////

            i += 2; // a hinge joint constraint is of size 2
        }

        for(const auto &f : fixed) {

            //////////////////////////
            // your code for Task 1.3
            //////////////////////////

            i += 2; // a fixed joint constraint is of size 2
        }

        if(fixedAngleEnabled)
            for(const auto &f : fixedAngle) {

                //////////////////////////
                // your code for Task 1.3
                //////////////////////////

                i += 1; // a fixed angle joint constraint is of size 1
            }

        return dCdx;
    }

    int getNumConstraints() const {
        return hingeJoints.size()*2 + fixed.size()*2
                + ((fixedAngleEnabled) ? fixedAngle.size()*1 : 0);
    }

public:
    std::vector<RigidBody> rigidbodies;
    std::vector<HingeJoint> hingeJoints;
    std::vector<FixedJoint> fixed;
    std::vector<FixedAngleJoint> fixedAngle;
    mutable bool fixedAngleEnabled = false;
};
