
//#include <Eigen/Core>
//using Eigen::Vector2f;
//using Eigen::Vector2d;
//using Eigen::VectorXd;

#ifdef WIN32
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <cmath>
#endif


#include <RigidBody.h>
#include <Joint.h>
#include <RigidBodySimulation.h>

#include <iostream>

struct TestResult {
    bool passed = true;
    std::string error = "";
};


TestResult testRigidBody() {

    RigidBody rb(0, 1.0);

    VectorXd x(3);
    x.setZero();
    x[2] = M_PI/4;

    const Vector2d p(1,1);
    Vector2d pWorld = rb.pWorld(x, p);
    if( (pWorld - Vector2d(0, sqrt(2))).norm() > 1e-8 )
        return TestResult({false, "p world wrong"});

    Matrix<double, 2, 3> dpWorld_dx = rb.dpWorld_dx(x, p);

    Matrix<double, 2, 3> dpWorld_dx_FD;
    dpWorld_dx_FD.setZero();

    const double dx = 1e-8;
    for (int i = 0; i < x.size(); ++i) {
        VectorXd xp = x;
        xp[i] += dx;
        VectorXd xm = x;
        xm[i] -= dx;

        Vector2d pWorld_p = rb.pWorld(xp, p);
        Vector2d pWorld_m = rb.pWorld(xm, p);

        dpWorld_dx_FD.block<2, 1>(0, i) = (pWorld_p - pWorld_m) / (2*dx);
    }

    if((dpWorld_dx - dpWorld_dx_FD).norm() > 1e-8)
        return TestResult({false, "dpWorld/dx wrong"});

    return TestResult();
}

TestResult testHingeJoint() {

    VectorXd x(6);
    x.setZero();

    x[3] = 1.;

    RigidBody rb1(0, 1.0);
    RigidBody rb2(3, 1.0);

    HingeJoint joint(0, 3, {0.5, 0}, {-0.5, 0.});

    MatrixXd dC_dx = joint.jacobian(x, rb1, rb2);

    MatrixXd dC_dx_FD(2, 6);
    const double dx = 1e-8;
    for (int i = 0; i < x.size(); ++i) {
        VectorXd xp = x;
        xp[i] += dx;
        VectorXd xm = x;
        xm[i] -= dx;

        VectorXd cp = joint.constraints(xp, rb1, rb2);
        VectorXd cm = joint.constraints(xm, rb1, rb2);
        dC_dx_FD.block<2, 1>(0, i) = (cp-cm) / (2*dx);
    }

    if((dC_dx - dC_dx_FD).norm() > 1e-8)
        return TestResult({false, "dC/dx wrong"});

    return TestResult();
}


TestResult testFixedJoint() {

    VectorXd x(3);
    x.setZero();

    RigidBody rb(0, 1.0);

    FixedJoint joint({{0.5, 0}, 0, {0.5, 0.5}});

    MatrixXd dC_dx = joint.jacobian(x, rb);

    MatrixXd dC_dx_FD(2, 3);
    const double dx = 1e-8;
    for (int i = 0; i < x.size(); ++i) {
        VectorXd xp = x;
        xp[i] += dx;
        VectorXd xm = x;
        xm[i] -= dx;

        VectorXd cp = joint.constraints(xp, rb);
        VectorXd cm = joint.constraints(xm, rb);
        dC_dx_FD.block<2, 1>(0, i) = (cp-cm) / (2*dx);
    }

    if((dC_dx - dC_dx_FD).norm() > 1e-8)
        return TestResult({false, "dC/dx wrong"});

    return TestResult();
}

TestResult testFixedAngleJoint() {

    VectorXd x(3);
    x.setZero();

    RigidBody rb(0, 1.0);

    FixedAngleJoint joint{0, M_PI/3};

    MatrixXd dC_dx = joint.jacobian(x, rb);

    MatrixXd dC_dx_FD(1, 3);
    const double dx = 1e-8;
    for (int i = 0; i < x.size(); ++i) {
        VectorXd xp = x;
        xp[i] += dx;
        VectorXd xm = x;
        xm[i] -= dx;

        VectorXd cp = joint.constraints(xp, rb);
        VectorXd cm = joint.constraints(xm, rb);
        dC_dx_FD.block<1, 1>(0, i) = (cp-cm) / (2*dx);
    }

    if((dC_dx - dC_dx_FD).norm() > 1e-8)
        return TestResult({false, "dC/dx wrong"});

    return TestResult();
}

TestResult testKinematicEnergy() {
    RigidBodySimulation sim;

    sim.rigidbodies().emplace_back(0, 1.2);
    sim.rigidbodies().emplace_back(3, 2.2);
    sim.rigidbodies().emplace_back(6, 2.2);

    VectorXd x(sim.rigidbodies().size() * 3);
    x << -0.0179025,
    0.499483,
    1.60662,
    0.836155,
    1.48865,
    0.511597,
    1.85407,
    0.989036,
    -1.42431;

    sim.hingeJoints().emplace_back(0, 1, Vector2d(0.5, 0), Vector2d(-1, 0));
    sim.hingeJoints().emplace_back(1, 2, Vector2d(1, 0), Vector2d(-1, 0));
    sim.fixed().push_back(FixedJoint({Vector2d(0,0), 0, Vector2d(-0.5, 0)}));
    sim.fixed().push_back(FixedJoint({Vector2d(2,0), 2, Vector2d(1, 0)}));

    MatrixXd dCdx = sim.energy.jacobian(x);

    const int nC = sim.energy.getNumConstraints();
    MatrixXd dC_dx_FD(nC, x.size());
    const double dx = 1e-8;
    for (int i = 0; i < x.size(); ++i) {
        VectorXd xp = x;
        xp[i] += dx;
        VectorXd xm = x;
        xm[i] -= dx;

        VectorXd cp = sim.energy.constraints(xp);
        VectorXd cm = sim.energy.constraints(xm);
        dC_dx_FD.block(0, i, nC, 1) = (cp-cm) / (2*dx);
    }

    if((dCdx - dC_dx_FD).norm() > 1e-6)
        return TestResult({false, "dC/dx wrong"});

    return TestResult();
}

typedef TestResult (*func_t)();
inline void test_function(func_t f, const char *name) {
    TestResult tr = f();
    if(!tr.passed){
        std::cerr << name << " failed: `" << tr.error << "`" << std::endl;
    }
    else {
        std::cout << name << ": [PASSED]" << std::endl;
    }
}

int main(int, char**)
{
    test_function(testRigidBody, "Task 1.1: dpWorld_dx");
    test_function(testHingeJoint, "Task 1.2: hinge joint Jacobian");
    test_function(testFixedJoint, "Task 1.2: fixed joint Jacobian");
    test_function(testFixedAngleJoint, "Task 1.2: fixed angle joint Jacobian");
    test_function(testKinematicEnergy, "Task 1.3: kinematic energy Jacobian");

    return 0;
}
