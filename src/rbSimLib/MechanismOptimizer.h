#pragma once

#include <utility>
#include "RigidBodySimulation.h"
#include <RandomMinimizer.h>

class MatchTrajectoryObjective : public ObjectiveFunction
{
public:
    MatchTrajectoryObjective(RigidBodySimulation sim, Matrix<double, -1, 2> targetPath)
        : sim(std::move(sim)), targetTrajectory(std::move(targetPath)) {}

     double evaluate(const VectorXd& p) const override {
        sim.setDesignParameters(p);
        Matrix<double, -1, 2> trajectory = sim.recordTrajectory();
        assert(trajectory.rows() == targetTrajectory.rows());

        double energy = 0;
#ifdef SOL_2
        energy = (trajectory - targetTrajectory).squaredNorm();
#endif
        return 0;
    }

public:
    mutable RigidBodySimulation sim;
    Matrix<double, -1, 2> targetTrajectory;
};

class MechanismOptimizer
{
public:
    MechanismOptimizer() = default;

    MechanismOptimizer(const RigidBodySimulation &sim) : sim(sim) {
        p = sim.getDesignParameters();
    }

    void optimizeTrajectory() {
        MatchTrajectoryObjective obj(sim, targetPath);
        minimizer.searchDomainMin = 0.95 * p;
        minimizer.searchDomainMax = 1.05 * p;
        minimizer.minimize(&obj, p);
    }

public:
    RigidBodySimulation sim;
    Matrix<double, -1, 2> targetPath;

    RandomMinimizer minimizer;
    VectorXd p;

};
