// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"


// using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses std::bind() to register a
//  * member function as a callback from the timer. */

// class GraphLocation : public rclcpp::Node
// {
// public:
//   GraphLocation()
//   : Node("minimal_publisher"), count_(0)
//   {
//     publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//     timer_ = this->create_wall_timer(
//       500ms, std::bind(&GraphLocation::timer_callback, this));
//   }

// private:
//   void timer_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hello, world! " + std::to_string(count_++);
//     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_->publish(message);
//   }
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//   size_t count_;
// };


// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<GraphLocation>());
//   rclcpp::shutdown();
//   return 0;
// }




#include <gtsam/geometry/Rot2.h>

// Each variable in the system (poses) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// We will apply a simple prior on the rotation. We do so via the `addPrior` convenience
// method in NonlinearFactorGraph.

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


using namespace std;
using namespace gtsam;

const double degree = M_PI / 180;

int main() {
  /**
   *    Step 1: Create a factor to express a unary constraint
   * The "prior" in this case is the measurement from a sensor,
   * with a model of the noise on the measurement.
   *
   * The "Key" created here is a label used to associate parts of the
   * state (stored in "RotValues") with particular factors.  They require
   * an index to allow for lookup, and should be unique.
   *
   * In general, creating a factor requires:
   *  - A key or set of keys labeling the variables that are acted upon
   *  - A measurement value
   *  - A measurement model with the correct dimensionality for the factor
   */
  Rot2 prior = Rot2::fromAngle(30 * degree);
  prior.print("goal angle");
  auto model = noiseModel::Isotropic::Sigma(1, 1 * degree);
  Symbol key('x', 1);

  /**
   *    Step 2: Create a graph container and add the factor to it
   * Before optimizing, all factors need to be added to a Graph container,
   * which provides the necessary top-level functionality for defining a
   * system of constraints.
   *
   * In this case, there is only one factor, but in a practical scenario,
   * many more factors would be added.
   */
  NonlinearFactorGraph graph;
  graph.addPrior(key, prior, model);
  graph.print("full graph");

  /**
   *    Step 3: Create an initial estimate
   * An initial estimate of the solution for the system is necessary to
   * start optimization.  This system state is the "RotValues" structure,
   * which is similar in structure to a STL map, in that it maps
   * keys (the label created in step 1) to specific values.
   *
   * The initial estimate provided to optimization will be used as
   * a linearization point for optimization, so it is important that
   * all of the variables in the graph have a corresponding value in
   * this structure.
   *
   * The interface to all RotValues types is the same, it only depends
   * on the type of key used to find the appropriate value map if there
   * are multiple types of variables.
   */
  Values initial;
  initial.insert(key, Rot2::fromAngle(20 * degree));
  initial.print("initial estimate");

  /**
   *    Step 4: Optimize
   * After formulating the problem with a graph of constraints
   * and an initial estimate, executing optimization is as simple
   * as calling a general optimization function with the graph and
   * initial estimate.  This will yield a new RotValues structure
   * with the final state of the optimization.
   */
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("final result");

  return 0;
}