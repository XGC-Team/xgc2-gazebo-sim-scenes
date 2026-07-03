#ifndef OBSTACLEPATHPLUGIN_HH
#define OBSTACLEPATHPLUGIN_HH
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen>

namespace gazebo
{
  class DynamicObstacle : public ModelPlugin{
  private:
  	// Gazebo stuff
  	physics::ModelPtr model; // Pointer to the model
  	event::ConnectionPtr updateConnection; // Pointer to the update event connection

  	// custom attr
  	sdf::ElementPtr sdf;
  	double velocity;
  	bool orientation;
    bool loop;
    bool sinWave;
    double angularVelocity;

  	std::vector<ignition::math::Vector3d> path;
  	std::vector<std::vector<double>> pathWithAngle;
  	std::vector<double> timeKnot;

  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void linear(const double &ts, const std::vector<std::vector<double>> &path, const std::vector<double> &time, std::vector<std::vector<double>> &traj, std::vector<double> &t);
    void sine(const double &ts, const std::vector<std::vector<double>> &path, const std::vector<double> &time, std::vector<std::vector<double>> &traj, std::vector<double> &t);
    void lls(const double &ts, const std::vector<std::vector<double>> &path, const std::vector<double> &time, std::vector<std::vector<double>> &traj, std::vector<double> &t, const int &order);
    Eigen::MatrixXd fit(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y, const Eigen::MatrixXd &px, const int &order);
    Eigen::MatrixXd vandermonde(const Eigen::MatrixXd &x, const int &order);
    std::vector<double>& interpolateAngle(double start, double end, double dx);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DynamicObstacle)
}

#endif