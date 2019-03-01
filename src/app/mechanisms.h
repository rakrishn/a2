#pragma once

#include <RigidBodySimulation.h>

static RigidBodySimulation make4barSim() {

    RigidBodySimulation sim;

    auto connect = [&](int rbIdx0, double l0, int rbIdx1, double l1){
        sim.hingeJoints().emplace_back(rbIdx0, rbIdx1, Vector2d(l0*sim.rigidbodies()[rbIdx0].length/2, 0), Vector2d(l1*sim.rigidbodies()[rbIdx1].length/2, 0));
    };

    auto fix = [&](Vector2d p, int rbIdx, double l){
            sim.fixed().emplace_back(FixedJoint{p, rbIdx, {sim.rigidbodies()[rbIdx].length/2 * l, 0}});
    };
    sim.rigidbodies().emplace_back(0, 3);
    sim.rigidbodies().emplace_back(3, 5);
    sim.rigidbodies().emplace_back(6, 7);
    sim.x.resize(sim.rigidbodies().size() * 3);
    sim.x <<
                 -4.62484,
                 -3.54767,
                    1.318,
                 -1.92376,
                 -1.17877,
                 0.375387,
                  1.20108,
                 -2.63109,
                 -1.24552;
    sim.x0 = sim.x;
    connect(0, 1, 1, -1);
    connect(1, 1, 2, -1);
    fix(Vector2d(-5, -5), 0, -1);
    fix(Vector2d(2, -5), 2, 1);

    sim.trackRBPoint.rbIdx = 1;
    sim.trackRBPoint.local = Vector2d(sim.rigidbodies()[1].length/2, 0);

    sim.fixedAngle().push_back(FixedAngleJoint({0, 0}));
    sim.motorIdx = 0;

    return sim;
}

static RigidBodySimulation makeJansenSim() {

    RigidBodySimulation sim;

    std::vector<double> jansen_dimensions = {
        38, 41.5, 39.3, 40.1, 55.8 ,39.4, 36.7, 65.7, 49, 50, 61.9, 7.8, 30
    };
    char letter[] = "a";
    std::map<std::string, int> idx;
    for (int i = 0; i < jansen_dimensions.size(); ++i) {
        RigidBody rb(3*i, jansen_dimensions[i]/10, std::string(letter));
        sim.rigidbodies().push_back(rb);
        idx[std::string(letter)] = sim.rigidbodies().size()-1;
        *letter += 1;
    }
    sim.x.resize(sim.rigidbodies().size() * 3);
    sim.x.setZero();
    sim.x <<
                 1.4896e-07		 ,
                -2.20021e-07	 ,
                 1.15252e-07	 ,
                  -1.20068		 ,
                      1.9536	 ,
                     1.22704	 ,
                  -1.34761		 ,
                  -1.88576		 ,
                  -1.28584		 ,
                  -3.73972		 ,
                  0.797155		 ,
                -0.408882		 ,
                   -3.0404		 ,
                     2.75076	 ,
                  0.427391		 ,
                   -4.8013		 ,
                   -0.215496	 ,
                  -1.16473		 ,
                  -2.40919		 ,
                  -2.89841		 ,
                 -0.495883		 ,
                  -3.21958		 ,
                   -5.2105		 ,
                  -1.32367		 ,
                  -1.60561		 ,
                  -6.08361		 ,
                     1.23367	 ,
                     1.44932	 ,
                      2.3436	 ,
                 -0.675698		 ,
                     1.30239	 ,
                  -1.49576		 ,
                  0.826112		 ,
                         1.9	 ,
                        0.39	 ,
                      1.5708	 ,
                         1.9	 ,
                        0.78	 ,
                -6.56333e-08;
    sim.x0 = sim.x;

    auto rb = [&](std::string s){
        return sim.rigidbodies()[idx.at(s)];
    };

    auto fix_rb_at = [&](std::string s, double local){
        sim.fixed().push_back(FixedJoint(
            {rb(s).pWorld(sim.x, Vector2d(local, 0)), idx.at(s), Vector2d(local, 0)}));
    };

    fix_rb_at("a", -rb("a").length/2);
    fix_rb_at("a",  rb("a").length/2);
    fix_rb_at("l", -rb("l").length/2);
    fix_rb_at("l",  rb("l").length/2);

    auto connect = [&](std::string s1, double l1, std::string s2, double l2){
        sim.hingeJoints().push_back(HingeJoint(idx[s1], idx[s2], Vector2d(l1*rb(s1).length/2, 0), Vector2d(l2*rb(s2).length/2, 0)));
    };

    connect("l",  1, "m", 0);
    connect("m",  1, "j",  1);
    connect("j", -1, "e",  1);
    connect("j", -1, "b",  1);
    connect("e", -1, "d", -1);
    connect("d",  1, "b", -1);
    connect("b", -1, "a", -1);
    connect("b", -1, "c", -1);
    connect("c",  1, "g",  1);
    connect("g", -1, "f",  1);
    connect("f", -1, "d", -1);
    connect("f",  1, "h", -1);
    connect("h",  1, "i", -1);
    connect("g",  1, "i",  1);
    connect("g",  1, "c",  1);
    connect("c",  1, "k", -1);
    connect("k",  1, "m", 1);

    sim.fixedAngle().push_back(FixedAngleJoint({12, 0}));
    sim.motorIdx = 0;

    sim.trackRBPoint.rbIdx = idx["h"];
    sim.trackRBPoint.local = Vector2d(rb("h").length/2, 0);

    return sim;
}

Matrix<double, -1, 2> makeJansenTargetPath() {
    Matrix<double, -1, 2> path(100, 2);
    path <<   -2.74293,   -8.34259,
                        -2.60577,   -8.34368,
                        -2.46477,   -8.34701,
                        -2.31976,   -8.35251,
                        -2.17065,   -8.36004,
                        -2.01745,   -8.36934,
                        -1.86038,   -8.38007,
                        -1.69974,   -8.39181,
                          -1.536,   -8.40412,
                        -1.36969,   -8.41655,
                        -1.20142,   -8.42866,
                        -1.03182,   -8.44005,
                       -0.861576,   -8.45037,
                       -0.691344,   -8.45932,
                         -0.5218,   -8.46667,
                       -0.353613,   -8.47222,
                        -0.18745,   -8.47586,
                      -0.0239805,   -8.47748,
                        0.136123,   -8.47707,
                        0.292177,   -8.47464,
                        0.443483,   -8.47023,
                        0.589321,   -8.46394,
                        0.728947,   -8.45588,
                        0.861582,   -8.44623,
                        0.986415,   -8.43516,
                         1.10259,   -8.42289,
                          1.2092,   -8.40965,
                         1.30529,   -8.39569,
                         1.38986,   -8.38127,
                         1.46182,   -8.36663,
                         1.52005,   -8.35204,
                         1.56335,   -8.33769,
                         1.59037,   -8.32379,
                         1.59993,   -8.31042,
                         1.59059,   -8.29761,
                         1.56093,   -8.28529,
                         1.50953,   -8.27322,
                           1.435,     -8.261,
                         1.33604,   -8.24804,
                         1.21147,    -8.2335,
                         1.06041,   -8.21632,
                         0.88231,   -8.19522,
                        0.677163,   -8.16875,
                        0.445629,   -8.13541,
                        0.189208,   -8.09378,
                       -0.089642,   -8.04282,
                       -0.387441,   -7.98223,
                        -0.69977,   -7.91277,
                        -1.02152,   -7.83672,
                        -1.34736,   -7.75808,
                        -1.67234,   -7.68247,
                        -1.99249,   -7.61654,
                        -2.30515,   -7.56672,
                        -2.60882,   -7.53784,
                         -2.9026,   -7.53192,
                        -3.18561,   -7.54793,
                        -3.45665,   -7.58239,
                        -3.71415,   -7.63059,
                        -3.95637,   -7.68766,
                        -4.18163,   -7.74931,
                        -4.38848,   -7.81218,
                        -4.57578,   -7.87388,
                        -4.74281,   -7.93287,
                        -4.88921,   -7.98826,
                          -5.015,   -8.03967,
                        -5.12052,   -8.08702,
                        -5.20639,   -8.13043,
                        -5.27341,   -8.17011,
                        -5.32255,   -8.20631,
                        -5.35484,   -8.23929,
                         -5.3714,   -8.26926,
                        -5.37342,   -8.29639,
                        -5.36189,   -8.32089,
                        -5.33797,   -8.34285,
                        -5.30274,   -8.36236,
                         -5.2572,    -8.3795,
                        -5.20231,   -8.39432,
                          -5.139,   -8.40687,
                        -5.06811,   -8.41721,
                        -4.99043,   -8.42538,
                        -4.90672,   -8.43144,
                        -4.81766,   -8.43546,
                        -4.72387,   -8.43753,
                        -4.62595,   -8.43775,
                         -4.5244,   -8.43622,
                        -4.41969,   -8.43309,
                        -4.31225,    -8.4285,
                        -4.20243,   -8.42264,
                        -4.09053,   -8.41569,
                        -3.97678,   -8.40787,
                        -3.86137,   -8.39942,
                        -3.74442,   -8.39061,
                        -3.62595,    -8.3817,
                        -3.50595,     -8.373,
                         -3.3843,   -8.36481,
                        -3.26083,   -8.35746,
                         -3.1353,   -8.35127,
                        -3.00739,   -8.34654,
                        -2.87674,   -8.34356,
                        -2.74299,   -8.34255;

    return path;
}
