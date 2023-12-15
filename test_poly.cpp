#include <utility>
#include <vector>
#include <iostream>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include "mav_trajectory_generation/segment.h"
#include <mav_trajectory_generation/impl/polynomial_optimization_linear_impl.h>
#include "mav_trajectory_generation/convolution.h"
#include <mav_trajectory_generation/segment.h>
#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation/polynomial.h>

#include "mav_trajectory_generation/extremum.h"
#include "mav_trajectory_generation/motion_defines.h"
#include "mav_trajectory_generation/polynomial.h"
#include "mav_trajectory_generation/trajectory.h"
#include "mav_trajectory_generation/vertex.h"

namespace mav = mav_trajectory_generation;
using namespace mav;

// 1. Create a list of three (x,y,z) vertices to fly through,
int main() {

    // mav_trajectory_generation::Vertex::Vector vertices;
    mav::Vertex::Vector vertices;
    const int dimension = 2;
    const int derivative_to_optimize = mav::derivative_order::SNAP;
    mav::Vertex start(dimension), middle(dimension), end(dimension);

    // 2. Add constraints to the vertices.
    start.makeStartOrEnd(Eigen::Vector2d(0,4.3), derivative_to_optimize);
    vertices.push_back(start);

    // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector2d(1,2));
    // vertices.push_back(middle);

    end.makeStartOrEnd(Eigen::Vector2d(50,4.3), derivative_to_optimize);
    vertices.push_back(end);


    // 3. Compute segment times
    std::vector<double> segment_times;
    const double v_max = 2.0;
    const double a_max = 2.0;
    segment_times = mav::estimateSegmentTimes(vertices, v_max, a_max);
    for (int i = 0; i < segment_times.size(); ++i) {
        std::cout << segment_times[i] << std::endl;
    }


    // 4. Optimize
    const int N = 10;
    mav::PolynomialOptimization<N> opt(dimension);

    // std::cout <<  "Base coefficients: " << mav::Polynomial::base_coefficients_.size() << std::endl;
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    std::cout << "Solve linear" << std::endl;
    opt.solveLinear();
    std::cout << "Done solving linear" << std::endl;

    // opt.printReorderingMatrix(std::cout);




    // 5. Obtain the polynomial segments.
    Segment::Vector segments;
    opt.getSegments(&segments);


    std::cout << segments << std::endl;
    std::cout << "Hello" << std::endl;

    return 0;
}
