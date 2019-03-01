#pragma once

#include "ObjectiveFunction.h"
#include "Minimizer.h"

class GradientDescentFixedStep : public Minimizer {
public:
    GradientDescentFixedStep(int maxIterations=100, double solveResidual=1e-5)
        : maxIterations(maxIterations), solveResidual(solveResidual) {
    }

    int getLastIterations() { return lastIterations; }

    virtual bool minimize(const ObjectiveFunction *function, VectorXd &x) const {

        bool optimizationConverged = false;

        VectorXd dx(x.size());

        int i=0;
        for(; i < maxIterations; i++) {
            dx.setZero();
            computeSearchDirection(function, x, dx);

            if (dx.norm() < solveResidual){
                optimizationConverged = true;
                break;
            }

            step(function, dx, x);
        }

        lastIterations = i;

        return optimizationConverged;
    }

protected:
    virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) const {
        function->addGradientTo(x, dx);
    }

    // Given the objective `function` and the search direction `x`, update the candidate `x`
    virtual void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x) const
    {
        x -= stepSize * dx;
    }

public:
    double solveResidual = 1e-5;
    int maxIterations = 1;
    double stepSize = 0.001;

    // some stats about the last time `minimize` was called
    mutable int lastIterations = -1;
};


class GradientDescentLineSearch : public GradientDescentFixedStep {
public:
    GradientDescentLineSearch(int maxIterations=100, double solveResidual=1e-5, int maxLineSearchIterations=15)
        : GradientDescentFixedStep (maxIterations, solveResidual), maxLineSearchIterations(maxLineSearchIterations){
    }

protected:
    void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x) const override
    {
        double alpha = 1.0;
        VectorXd xc(x);
        double initialValue = function->evaluate(xc);

        for(int j = 0; j < maxLineSearchIterations; j++) {
            x = xc - dx * alpha;

            double newLineSearchValue = function->evaluate(x);

            if((!std::isfinite(newLineSearchValue) || newLineSearchValue > initialValue)
                    && j < maxLineSearchIterations -1)
                alpha /= 2.0;
            else
                return;
        }
    }

protected:
    int maxLineSearchIterations = 15;
};
