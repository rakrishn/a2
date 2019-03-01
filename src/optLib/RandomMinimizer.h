#pragma once

#include "Minimizer.h"

#include <random>

class RandomMinimizer : public Minimizer
{
public:
    RandomMinimizer(const VectorXd &upperLimit = VectorXd(), const VectorXd &lowerLimit = VectorXd(), double fBest = HUGE_VAL)
        : searchDomainMax(upperLimit), searchDomainMin(lowerLimit), fBest(fBest) {
        fBest = HUGE_VAL;

        // initialize random number generator
        rng.seed(std::random_device()());
        dist = std::uniform_real_distribution<>(0.0,1.0);
    }

    virtual ~RandomMinimizer() {}

    bool minimize(const ObjectiveFunction *function, VectorXd &x) const override {
        for (int i = 0; i < iterations; ++i) {

            // generate a random number in [0, 1]
            // double a = dist(rng);

            ////////////////////////////////////// 1

            VectorXd xr(x.size());
            for (int i = 0; i < x.size(); ++i) {
                xr[i] = dist(rng) * (searchDomainMax[i] - searchDomainMin[i]) + searchDomainMin[i];
            }

            double f = function->evaluate(xr);
            if(f < fBest){
                x = xr;
                fBest = f;
            }

            ////////////////////////////////////// 1
        }
        return false;
    }

public:
    int iterations = 1;
    VectorXd searchDomainMax, searchDomainMin;
    mutable double fBest;

    mutable std::uniform_real_distribution<double> dist;
    mutable std::mt19937 rng;
};
