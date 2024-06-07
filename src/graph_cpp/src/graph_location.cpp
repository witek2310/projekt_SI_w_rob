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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ros includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// gtsam includes
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class GraphLocation : public rclcpp::Node
{
public:
	GraphLocation()
		: Node("graph_location")
	{
        // subscribers declaration
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&GraphLocation::odom_callback, this, std::placeholders::_1));
		imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&GraphLocation::imu_callback, this, std::placeholders::_1));
		laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("laser", 10, std::bind(&GraphLocation::laser_callback, this, std::placeholders::_1));
		amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&GraphLocation::amcl_callback, this, std::placeholders::_1));

        // publisher declaration
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
		publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/graph_pose", 10);//qos);

		// initialize graph
		graph_ = new NonlinearFactorGraph();

		// Assemble initial quaternion through GTSAM constructor
		// ::quaternion(w,x,y,z);
		Rot3 prior_rotation = Rot3::Quaternion(0.0, 0.0, 0.0, 0.0);
		Point3 prior_point(0.0, 0.0, 0.0);
		Pose3 prior_pose(prior_rotation, prior_point);
		Vector3 prior_velocity(0.0, 0.0, 0.0);
		imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
		this->prev_bias_ = prior_imu_bias;
		this->prev_state_ = NavState(prior_pose, prior_velocity);
		iteration_ = 0;
		
		initial_values_.insert(X(iteration_), prior_pose);
		initial_values_.insert(V(iteration_), prior_velocity);
		initial_values_.insert(B(iteration_), prior_imu_bias);

		// Assemble prior noise model and add it the graph.`
		auto pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
		auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);												 // m/s
		auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

		// Add all prior factors (pose, velocity, bias) to the graph.
		this->graph_->addPrior(X(iteration_), prior_pose, pose_noise_model);
		this->graph_->addPrior(V(iteration_), prior_velocity, velocity_noise_model);
		this->graph_->addPrior(B(iteration_), prior_imu_bias, bias_noise_model);

		auto p = imuParams();

		preintegrated_ = std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);
		if (!preintegrated_) {
			RCLCPP_ERROR(this->get_logger(), "Failed to allocate PreintegratedImuMeasurements.");
			return;
   		 }
		assert(preintegrated_);

		RCLCPP_INFO(this->get_logger(), "innit done");
		prev_imu_time_ = 0.0;
		// // Store previous state for imu integration and latest predicted outcome.
		// NavState prev_state(prior_pose, prior_velocity);
		// NavState prop_state = prev_state;
		// imuBias::ConstantBias prev_bias = prior_imu_bias;

		// // Keep track of total error over the entire run as simple performance metric.
		// double current_position_error = 0.0, current_orientation_error = 0.0;

		// double output_time = 0.0;
		// double dt = 0.005;  // The real system has noise, but here, results are nearly
		// 					// exactly the same, so keeping this for simplicity.

		// All priors have been set up, now iterate through the data file.
	}
	~GraphLocation(){
		delete this->graph_;
	}

private:
	// variables for publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

	// basic operation, graph
	int iteration_;
	NonlinearFactorGraph *graph_;
	std::shared_ptr<PreintegratedImuMeasurements> preintegrated_;
	Values initial_values_;
	imuBias::ConstantBias prev_bias_;
	double prev_imu_time_;
	Pose3 prev_amcl_; 
	NavState prev_state_;

	// variables for ros2 messages
	bool odom_recived_, imu_recived_, laser_recived_;
	sensor_msgs::msg::Imu imu_msg_;
	sensor_msgs::msg::LaserScan laser_msg_;
	Pose3 odom_msg_, prev_odom_msg_;
	geometry_msgs::msg::PoseWithCovarianceStamped amcl_msg_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
	
	void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
	{
		// RCLCPP_INFO(this->get_logger(), "I heard: xd'");

		Rot3 temp_rot = Rot3::Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
		Point3 temp_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
		Pose3 temp_pose(temp_rot, temp_position);

		this->odom_msg_ = temp_pose;
	}
	void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
	{	
    RCLCPP_INFO(this->get_logger(), "Imucallback start");

    try {
        double time = (double)msg->header.stamp.sec + ((double)msg->header.stamp.nanosec) * 0.000000001;
        if (this->prev_imu_time_ == 0.0) {
            this->prev_imu_time_ = time;
            RCLCPP_INFO(this->get_logger(), "Initial IMU time set: %f", time);
            return; // Skip the first IMU message as we have no previous time to compare
        }

        double dt = time - this->prev_imu_time_;
        if (dt <= 0) {
            RCLCPP_WARN(this->get_logger(), "Non-positive dt detected: %f", dt);
            return; // Skip this IMU message
        }

        this->imu_msg_ = *msg;
        this->imu_recived_ = true;

        RCLCPP_INFO(this->get_logger(), "Time: %f, Previous IMU time: %f, dt: %f", time, this->prev_imu_time_, dt);

        Vector3 imu_acc = Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Vector3 imu_ome = Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        RCLCPP_INFO(this->get_logger(), "IMU Acc: [%f, %f, %f], IMU Omega: [%f, %f, %f]", imu_acc.x(), imu_acc.y(), imu_acc.z(), imu_ome.x(), imu_ome.y(), imu_ome.z());

        this->preintegrated_->integrateMeasurement(imu_acc, imu_ome, dt);

        this->prev_imu_time_ = time;

        RCLCPP_INFO(this->get_logger(), "Imucallback end");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in IMU callback: %s", e.what());
    }
	
	}
	void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		this->laser_msg_ = *msg;
		this->laser_recived_ = true;
	}
	void amcl_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		// since the amcl node has low public
		// crating a position node from amcl and noise form topic
		Rot3 rotation_amcl = Rot3::Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
		Point3 position_amcl(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
		Pose3 pose_amcl(rotation_amcl, position_amcl);
		
		//initial valuse are amcls' values
		this->initial_values_.insert(X(iteration_ + 1), pose_amcl);

		//adding the factor between X_n and X_n+1 based on amcl measurements 
        // size_t covariance_length = sizeof(msg->pose.covariance);
        // RCLCPP_INFO(this->get_logger(), "Length of covariance array: %zu", covariance_length);

		auto amcl_noise = noiseModel::Diagonal::Sigmas(Vector6(msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[14], msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]));
		auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
        auto pose_amcl_change = this->prev_amcl_.between(pose_amcl);
		this->graph_->emplace_shared<BetweenFactor<Pose3>>(X(iteration_), X(iteration_ + 1), pose_amcl_change, amcl_noise);

		//adding the factor between X_n and X_n+1 based on odom measurements
		Pose3 pose_odom_change = this->prev_odom_msg_.between(this->odom_msg_);
  		auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector6(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
		this->graph_->emplace_shared<BetweenFactor<Pose3>>(X(iteration_), X(iteration_ + 1), pose_odom_change, odometryNoise);

		RCLCPP_INFO(this->get_logger(), "before imu");
		//adding the factor between X_n and X_n+1 based on imu measurements 
		auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated_);
		ImuFactor imu_factor(X(iteration_), V(iteration_),
							X(iteration_ +1 ), V(iteration_+ 1),
							B(iteration_), preint_imu);
		graph_->add(imu_factor);

        imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        graph_->add(BetweenFactor<imuBias::ConstantBias>(B(iteration_), B(iteration_ +1 ), zero_bias, bias_noise_model));

        RCLCPP_INFO(this->get_logger(), "added factor imu to graph");
        
        auto prop_state = preintegrated_->predict(this->prev_state_, this->prev_bias_);

        this->initial_values_.insert(V(iteration_+1), prop_state.v());
        this->initial_values_.insert(B(iteration_+1), this->prev_bias_);


        RCLCPP_INFO(this->get_logger(), "before getting to results");

        //getting the results
        auto result = LevenbergMarquardtOptimizer(*this->graph_, this->initial_values_).optimize();
        auto covariance = Marginals(*graph_, result).optimize();

        this->prev_bias_ = result.at<imuBias::ConstantBias>(B(iteration_ + 1));
        this->prev_state_ = NavState(result.at<Pose3>(X(iteration_+1)), result.at<Vector3>(V(iteration_+1)));

        RCLCPP_INFO(this->get_logger(), "before resetIntegrationAndSetBias");


		preintegrated_->resetIntegrationAndSetBias(this->prev_bias_);

		auto pose_graph = result.at<Pose3>(X(iteration_ + 1));
		

        // Making message to send by topic /graph_pose
		auto temp = geometry_msgs::msg::PoseWithCovarianceStamped();

        // header
        temp.header.stamp = this -> now();
        temp.header.frame_id = "map";

        // position
		temp.pose.pose.position.x = pose_graph.x();
		temp.pose.pose.position.y = pose_graph.y();
		temp.pose.pose.position.z = pose_graph.z();

        //orientation
		temp.pose.pose.orientation.x = pose_graph.rotation().toQuaternion().x();
		temp.pose.pose.orientation.y = pose_graph.rotation().toQuaternion().y();
		temp.pose.pose.orientation.z = pose_graph.rotation().toQuaternion().z();
		temp.pose.pose.orientation.w = pose_graph.rotation().toQuaternion().w();


        // setting covariance
        // auto imu_params = imuParams();
        // for (int i = 0; i < 9; ++i) 
        // {
        //     temp.pose.covariance[i] = imu_params->accelerometerCovariance(i / 3, i % 3); // Setting position covariance from accelerometer
        // }

        // for (int i = 0; i < 9; ++i) 
        // {
        //     temp.pose.covariance[21 + i] = imu_params->gyroscopeCovariance(i / 3, i % 3); // Setting orientation covariance from gyroscope
        // }

        // // Przykład ustawienia niektórych wartości kowariancji
        temp.pose.covariance[0] = 0.1;  // Kowariancja x-x
        temp.pose.covariance[7] = 0.1;  // Kowariancja y-y
        temp.pose.covariance[14] = 0.1; // Kowariancja z-z
        temp.pose.covariance[21] = 0.1; // Kowariancja rot_x-rot_x
        temp.pose.covariance[28] = 0.1; // Kowariancja rot_y-rot_y
        temp.pose.covariance[35] = 0.1; // Kowariancja rot_z-rot_z

        RCLCPP_INFO(this->get_logger(), "after adding position and location");



		this->publisher_->publish(temp);
		this->prev_odom_msg_ = this->odom_msg_;
		prev_amcl_ = pose_amcl;
		iteration_++;
	}

	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams()
	{
		// We use the sensor specs to build the noise model for the IMU factor.
		double accel_noise_sigma = 0.017;
		double gyro_noise_sigma = 0.000205689024915;
		double accel_bias_rw_sigma = 0.0;
		double gyro_bias_rw_sigma = 0.0;
		Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
		Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
		Matrix33 integration_error_cov =
			I_3x3 * 1e-8; // error committed in integrating position from velocities
		Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
		Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
		Matrix66 bias_acc_omega_init =
			I_6x6 * 1e-5; // error in the bias used for preintegration

		auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
		// PreintegrationBase params:
		p->accelerometerCovariance =
			measured_acc_cov; // acc white noise in continuous
		p->integrationCovariance =
			integration_error_cov; // integration uncertainty continuous
		// should be using 2nd order integration
		// PreintegratedRotation params:
		p->gyroscopeCovariance =
			measured_omega_cov; // gyro white noise in continuous
		// PreintegrationCombinedMeasurements params:
		p->biasAccCovariance = bias_acc_cov;	 // acc bias in continuous
		p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
		p->biasAccOmegaInt = bias_acc_omega_init;

		return p;
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GraphLocation>());
	rclcpp::shutdown();

	return 0;
}
