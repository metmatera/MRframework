#pragma once
#ifndef ArmbandProxy_HPP_
#define ArmbandProxy_HPP_



// System Header files
#include <map>
#include <vector>

// V-REP header files
extern "C" {
#include "extApi.h"
}

//Project Header files
#include "HapticProxy.hpp"

#include "RLS.hpp"

#include "Armband.hpp"

#include <iostream>
#include <string>
#include <sys/timeb.h>

#include <serial.h>
#include <HapticDevice.h>
#include <ctime>
#include <stdlib.h>

// Space dimension is 3...
#ifndef SPACE_DIM
#define SPACE_DIM 3
#endif // SPACE_DIM


class ArmbandPattern
{
public:

	/**
	* @brief Default contructor of ArmbandPatter class
	**/
	ArmbandPattern(int n_motors);

	/**
	* @brief Function to render vibration pattern
	**/
	void render(int pattern, const Eigen::VectorXf& f);

	/**
	* @brief Pattern1 rendering function
	**/
	void pattern1(
		Eigen::Vector4i& armBand_motorsi, Eigen::Vector3f friction, Eigen::Vector3f elastic, Eigen::Vector3f fb_m,
		float motor_force_fric, float motor_force_elas, float motor_force);

	/**
	* @brief Pattern2 rendering function
	**/
	void pattern2(
		Eigen::Vector4i& armBand_motorsi, Eigen::Vector3f friction, Eigen::Vector3f elastic, Eigen::Vector3f fb_m,
		float motor_force_fric, float motor_force_elas, float motor_force);

	/**
	* @brief Pattern3 rendering function
	**/
	void pattern3(
		Eigen::Vector4i& armBand_motorsi, Eigen::Vector3f friction, Eigen::Vector3f elastic, Eigen::Vector3f fb_m,
		float motor_force_fric, float motor_force_elas, float motor_force);

	/**
	* @brief Default destroyer of ArmbandPatterclass
	**/
	~ArmbandPattern();

private:

	// ------------ Custom Force Feedback variables ------------ //
	// Task variables
	int task_started;
	int k;
	bool start_penetration;
	int pattern_selected;

	// Pattern variables
	/// Pattern 2
	int div;
	bool is_puncturing;
	bool tmp_is_puncturing;
	bool is_transition;
	/// Pattern 3
	int num_motors;
	int layers_passed;
	bool uprising;
	bool layers_updated;
	bool p2_alt_solo;
	std::vector<int> motor_state;
	std::vector<float> motor_elastic_f;
	std::vector<float> motor_friction_f;
	std::vector<bool> penetrated;
	// -------------------------------------------------------------- //

};


class VibBrac
{
public:

	/**
	* @brief Default contructor of VibBrac class
	*
	*/
	VibBrac(int n);

	/* @brief Default destroyer of VibBrac class
	*
	*/
	~VibBrac();

};

class ArmbandProxy : public HapticProxy {//HLInterface, 

public:

// Original macro
#define MAX_FORCE 0.001
#define MAX_FRE_ARMBAND 205
#define MIN_FRE_ARMBAND 30
#define MAX_ERROR_THRESHOLD_LOWER_BOUND 0.01
#define MAX_ERROR_THRESHOLD_UPPER_BOUND 0.2
#define ARMBAND_OFFSET MIN_FRE_ARMBAND* MAX_ERROR_THRESHOLD_UPPER_BOUND/MAX_FRE_ARMBAND
#define MAX_Elastic_force 0.15f//( 0.010f +0.020f +0.015f +0.019f)
#define MAX_Elastic_force_armband_effect 255//-MAX_FRE_ARMBAND

// Pattern selection macro
#define F_PATTERN_1 1
#define F_PATTERN_2 2
#define F_PATTERN_3 3

// Other custom macro
#define DELTA_FR_FORCE 20		// Value to increase the friction force feedback
#define DELTA_EL_FORCE 40		// Value to increase the elastic force feedback
	
	/**
	* @brief Default contructor of ArmbandProxy class
	*
	*/
	ArmbandProxy();


	/* @brief Default destroyer of ArmbandProxy class
	*
	*/
	~ArmbandProxy();

	/**
	* @brief Copy constructor of the ArmbandProxy class
	* @param vp the ArmbandProxy
	*/
	ArmbandProxy(ArmbandProxy& vp);

	/**
	* @brief Default init function
	*/
	void init();

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default clear function
	*/
	void clear();

	/**
	 * @adding  cutting forces into tissues effect
	 */
	Eigen::Vector4i ArmbandProxy::Cutting_force2motors(int _ForcesAndTorques[], float Elastic_force);
	/**
	* @Converts the pure forces into PWM signalas to be sent until ArmBand device
	*/
	Eigen::Vector4i Forces2MotorsDim(Eigen::Vector6f);

	/**
	* @Send PWM to each motor's ArmBand according to the EF position, warning thrwow level frecuncy PWM  when the trajectory isno in a straigh forward  way(z-axis).
	*/
	Eigen::Vector4i Trajectory_alert(Eigen::Vector3f, Eigen::Vector3f, float Elastic_force);


	/**
	* @brief Debug function
	* Print the names of all the V-REP objects loaded for the simulation
	*/
	void print_Forces_Set(Eigen::Vector6f sensorFe0et);

	Eigen::Vector3f EF_Error;

	Eigen::Vector6f sensorFe0et;

	VibBrac* vibs;


	/**
	* @brief Set function
	* Set the feedback haptic force on the Armband device
	* @param f: the feedback force to be set
	*/
	void sendForce(const Eigen::VectorXf& f);

	/**
	* @brief CUSTOM Set function
	* Set the custom feedback haptic force on the Armband device
	* @param f: the feedback force to be set -> flat 3x3 matrix with shape (1,9)
	* @param armband_pattern: the ArmbandPattern class instance
	*/
	void sendForceCustom(const Eigen::VectorXf& f, ArmbandPattern& armband_pattern);


private:

	double time_;

};


#endif // VREPREMOTESERVICE_HPP_

