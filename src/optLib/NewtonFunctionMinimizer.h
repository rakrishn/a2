    #pragma once

#include "ObjectiveFunction.h"
#include "GradientDescentMinimizer.h"

class NewtonFunctionMinimizer : public GradientDescentLineSearch {
public:
    NewtonFunctionMinimizer(int maxIterations = 100, double solveResidual = 0.0001, int maxLineSearchIterations = 15)
        : GradientDescentLineSearch(maxIterations, solveResidual, maxLineSearchIterations) {	}

    virtual ~NewtonFunctionMinimizer() {}

protected:
    void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) const override {

        // get hessian
        function->getHessian(x, hessian);

        // add regularization
        hessian += VectorXd::Ones(x.size()).asDiagonal() * reg;

        // get gradient
        VectorXd gradient(x.size());
        gradient.setZero();
        function->addGradientTo(x, gradient);

        //dp = Hes^-1 * grad
        Eigen::SimplicialLDLT<SparseMatrixd, Eigen::Lower> solver;
        solver.compute(hessian);
        dx = solver.solve(gradient);
    }

public:
    std::vector<Triplet<double>> hessianEntries;
    double reg = 1e-5;

    mutable SparseMatrixd hessian;
};
