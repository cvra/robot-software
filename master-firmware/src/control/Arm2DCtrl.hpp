#pragma once

#include "Eigen/Dense"
#include "RefTracking.hpp"
#include "LQRController.hpp"

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

/* A class to control a robot arm in a 2D space
 *
 * Dimensions:
 * 		- m: cartesian coordinates
 *		- n: number of joints
 *		- p: number of control inputs
 *		- horizon: the LQR horizon
 * 
 * Parameters:
 *		- seg_len: length of each segments
 *		- lqr: A lqr setup based on LQR structure
 */
template <int m, int n, int p, int horizon>
class Arm2DCtrl {
public:
	Arm2DCtrl() = default;

	Arm2DCtrl(
		const Eigen::Matrix<float,n,1>& seg_len,
		const LQRController<n*2,p,horizon>& lqr)
	{
		l = seg_len;
		A = lqr.system.A;
		B = lqr.system.B;
		K = lqr.gain();
	}

	Arm2DCtrl(const Arm2DCtrl&) = default;
	Arm2DCtrl& operator=(const Arm2DCtrl&) = default;

	/* Set a new target position in the cartesian reference
	 * to tell the controller where to go
	 *
	 * Parameters:
	 *		- r: new target position
	 *		- a: maximum acceleration of the object in cartesian plan
	 *		- b: position of the base shoulder in cartesian plan
	 *		- q: angles position of each axe relative from each others
	 */
	void setNewTarget(
		const Eigen::Matrix<float, m, 1>& r, 
		const float a,
		const Eigen::Matrix<float, m, 1>& b,
		const Eigen::Matrix<float, n, 1>& q)
	{
		init_pos = getArmPosition(b,q);
		ref = RefTracking<m>(r-init_pos,a);
		reset_state();
	}

	/* Get the position in the cartesian plan of the end arm
	 *  
	 * Parameters:
	 *		- b: position of the base shoulder in cartesian plan
	 *		- q: angles position of each axe relative from each others		- 
	 *
	 * Return : 
	 * 		- Position of the end arm
	 */
	Eigen::Matrix<float, m, 1> getArmPosition(
		const Eigen::Matrix<float, m, 1>& b,
		const Eigen::Matrix<float, n, 1>& q)
	{
		Eigen::Matrix<float, n+1, m> pos;
		Eigen::Matrix<float, n, 1> q_world = q;
		
		pos.row(0) = b;
		for(int i=0; i<n; i++){
			pos(i+1,0) = pos(i,0) + l(i)*std::sin(q_world(i));
			pos(i+1,1) = pos(i,1) + l(i)*std::cos(q_world(i));
		}

		return pos.row(n);
	}

	Eigen::Matrix<float, n, 1>predicted_vel(
		const Eigen::Matrix<float, n, 1>& q,
		const float t,
		const float dt)
	{
		Eigen::Matrix<float, n, 1> dq_out;
		Eigen::Matrix<float, m, 1> tar_k_1;
		Eigen::Matrix<float, m, 1> tar_k_2;
		Eigen::Matrix<float, n, 1> q_k_1;
		Eigen::Matrix<float, n, 1> q_k_2;
		Eigen::Matrix<float, m, 1> b;
		b << 0., 0.;

		if(t < 0.){
			dq_out << 0.,0.,0.;
			return dq_out;
		}
		else if(t > 3*ref.t0){
			dq_out << 0.,0.,0.;
			return dq_out;
		}
		else{
			tar_k_1 = init_pos + ref.r(t);
			tar_k_2 = init_pos + ref.r(t+dt);
			q_k_1 = step(q,b,tar_k_1);
			q_k_2 = step(q_k_1,b,tar_k_2);
			for(int i=0; i<n;i++){
				dq_out[i] = angleDiff(q_k_2(i),q_k_1(i));
			}
			return dq_out/dt;
		}
	}

	/* Run the controller which coordonate the movement
	 *
	 * Parameters:
	 *		- b: position of the base shoulder in cartesian plan
	 *		- q: global state which is angular pos, velocity and acceleration
	 *		- dt: time spent since last call
	 * 
	 * Return:
	 * 		- The output q is the reference for the controller
	 *		  Ex: -B*K*q is the command for a LQR controller
	 */
	Eigen::Matrix<float, 2*n, 1> run(
		const Eigen::Matrix<float, 2*n, 1>& q,
		const Eigen::Matrix<float, m, 1>& b,
		const float dt)
	{
		Eigen::Matrix<float, n, 1> angles_in;
		Eigen::Matrix<float, n, 1> angles_out;
		Eigen::Matrix<float, n, 1> vel_out;
		Eigen::Matrix<float, 2*n, 1> q_out;
		Eigen::Matrix<float, m, 1> tar;

		t += dt;
		tar = init_pos + ref.r(t);

		angles_in(0) = q(0);
		angles_in(1) = q(2);
		angles_in(2) = q(4);
		q_out << q;

		vel_out = predicted_vel(angles_in,t,dt);
		angles_out = step(angles_in,b,tar);

		// fill angle output
		for(int i=0;i<n;i++){
			q_out(2*i) = angleDiff(angles_in(i),angles_out(i));
			q_out(2*i+1) = angleDiff(q(2*i+1),vel_out(i));
		}

		return q_out;
	}
	
	/* Make an iteration of the controller toward the reference target
	 *
	 * Parameters:
	 *		- b: position of the base shoulder in cartesian plan
	 *		- q: angles position of each axe relative from each others		
	 *		- tar: target position
	 * 		- tol: tolerance for the convergence of the optimizer
	 *
	 * Return:
	 *		- Angle position reference for the controller
	 */	
	Eigen::Matrix<float, n, 1> step(
		const Eigen::Matrix<float, n, 1>& q,
		const Eigen::Matrix<float, m, 1>& b,
		const Eigen::Matrix<float, m, 1>& tar,
		const float tol = 0.001)
	{

		Eigen::Matrix<float, n+1, m> pos;
		Eigen::Matrix<float, n, 1> q_world = q;
		pos.row(0) = b;

		for(int i=0; i<n; i++){
			pos(i+1,0) = pos(i,0) + l(i)*sinf(q_world(i));
			pos(i+1,1) = pos(i,1) + l(i)*cosf(q_world(i));
		}

		return invKinematic(pos, tar, tol);
	}

	/*
	 * Return: the norm a Matrix
	 */
	float norm(const Eigen::Matrix<float, m, 1>& a) 
	{
		return a.norm();
	}

	/*
	 * Return: the angle constrained in [-pi;pi]
	 */
	float constrainAngle(float x){
	    x = fmod(x + M_PI,2*M_PI);
	    if (x < 0)
	        x += 2*M_PI;
    	return x - M_PI;
	}

	/*
	 * Return: the shortest angle between angle a and b
	 */
	float angleDiff(const float a, const float b)
	{
	    float dif = fmod(a - b + M_PI,2*M_PI);
	    if (dif < 0)
	        dif += 2*M_PI;
	    return dif - M_PI;
	}

	/* The inverse Kinematic is the optimizer of the controller
	 * It computes the arm angle position of each joint from a reference
	 * in the cartesian plan
	 *
	 * Parameters:
	 *		- pos: the current cartesian position of each joint
	 *		- tar: the target position of the end joint
	 * 		- tol: a convergence tolerance
	 */
	Eigen::Matrix<float, n, 1> invKinematic(
		const Eigen::Matrix<float, n+1, m>& pos,
		const Eigen::Matrix<float, m, 1>& tar_t,
		const float tol = 0.001)
	{
		float dif;
		Eigen::Matrix<float, n+1, m> _p = pos;
		Eigen::Matrix<float, m, 1> b = _p.row(0);
		Eigen::Matrix<float, n, 1> r;
		Eigen::Matrix<float, n, 1> s;
		Eigen::Matrix<float, n, 1> q;

		const Eigen::Matrix<float, 1, m> tar = tar_t.transpose();
		
		if(std::abs(norm(tar_t-b)) > l.sum()){
			for(int i=0; i<n; i++){
				r(i) = norm(_p.row(i)-tar);
				s(i) = l(i)/r(i);
				_p.row(i+1) = (1-s(i))*_p.row(i) + s(i)*tar;
			}
		}
		else{
			dif = norm(_p.row(n)-tar);
			while(dif > tol){
				_p.row(n) = tar;
				for(int i=n-1; i>=0; i--){
					r(i) = norm(_p.row(i+1) - _p.row(i));
					s(i) = l(i)/r(i);
					_p.row(i) = (1-s(i))*_p.row(i+1) + s(i)*_p.row(i);
				}
				_p.row(0) = b;
				for(int i=0; i<n; i++){
					r(i) = norm(_p.row(i+1) - _p.row(i));
					s(i) = l(i)/r(i);
					_p.row(i+1) = (1-s(i))*_p.row(i) + s(i)*_p.row(i+1);
				}
				dif = norm(_p.row(n)-tar);
			}
		}

		for(int i=0; i<n; i++){
			q(i) = std::atan2(_p(i+1,0)-_p(i,0),_p(i+1,1)-_p(i,1));
		}
		return q;
	}

	class RefTracking<m> ref;
	Eigen::Matrix<float, n, 1> l;
	Eigen::Matrix<float, 2*n, 2*n> A;
	Eigen::Matrix<float, 2*n, p> B;
	std::array<Eigen::Matrix<float, p, 2*n>, horizon> K;

//private:

	void reset_state()
	{
		t = 0.;
	}

	float t;
	Eigen::Matrix<float, m, 1> init_pos;
};