#ifndef HAPTIC_INTERFACE_HPP_
#define HAPTIC_INTERFACE_HPP_

// Project Header files
#include "Instrument.hpp"

class HapticInterface : public Instrument {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	/**
	* @brief Default constructor of the HapticInterface class
	*/
	HapticInterface();

	/**
	* @brief Default constructor of the HapticInterface class
	*/
	HapticInterface(const std::string& name_);

	/**
	* @brief Default destroyer of the HapticInterface class
	*/
	~HapticInterface() {}

	/**
	* @brief Set function
	* Set the scale factor for the master commanded force on the slave system
	* @param vs the scale factor to be set
	*/
	inline void setForceScale(const float& fs) { this->force_scale = fs; }

	/**
	* @brief Get function
	* Get the scale factor for the master commanded force on the slave system
	* @return the scale factor to be retrieved
	*/
	inline float getForceScale() { return this->force_scale; }

	/**
	* @brief Set function
	* Set the 3D feedback force vector
	* @param f: the 3D feedback force vector to be set
	*/
	inline void setForceFeedback(const Eigen::VectorXf& f) { this->fbForce = f; }

	/**
	* @brief Get function
	* Get the 3D feedback force vector
	* @return the 3D feedback force vector
	*/
	inline Eigen::VectorXf getForceFeedback() { return this->fbForce; }

	/**
	* @brief Set function
	* Set the feedback pattern
	* @param f: feedback pattern to be set
	*/
	inline void setFeedbackPattern(const int& p) { this->fbPattern = p; }

	/**
	* @brief Get function
	* Get the feedback pattern
	* @return the feedback pattern
	*/
	inline int getFeedbackPattern() { return this->fbPattern; }

	/**
	* @brief Set function
	* Set the feedback rupture time registered by the user
	* @param rup_time: rupture time to be set
	*/
	inline void setFeedbackRuptureTime(const char * rup_time) { this->fbRuptureTime = (char *)rup_time; }

	/**
	* @brief Get function
	* Get the feedback rupture time
	* @return the feedback rupture time, registered after the user detection
	*/
	inline char * getFeedbackRuptureTime() { return this->fbRuptureTime; }

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);


	/**
	* @brief Check function
	* Check if the force feedback is enabled in the teleoperation control
	* @return true if the force feedback is enabled, false otherwise
	*/
	inline bool isTeleoperationWithForceFB() { return this->useForceFeedback; }

	/**
	* @brief Switch function
	* Switch the force feedback option on and off
	*/
	inline void swtichForceFeedback() { this->useForceFeedback = !this->useForceFeedback; }

	/**
	* @brief Set function
	* Set the rotation matrix expressing the orientation of the slave system (Kuka) wrt the master system (Geomagic)
	* @param rot the rotation matrix
	*/
	inline void setRms(const Eigen::MatrixXf& rot) { this->Rms = rot; }

	/**
	* @brief Get function
	* Get the rotation matrix expressing the orientation of the slave system (Kuka) wrt the master system (Geomagic)
	* @return the rotation matrix
	*/
	inline Eigen::MatrixXf getRms() { return this->Rms; }


protected:

	float force_scale;				//!< Scale factor for the master commanded force on the slave system
	Eigen::VectorXf fbForce;		//!< 3D feedback force vector of the Haptic interface
	int fbPattern;					//!< feedback pattern of the Haptic interface
	char * fbRuptureTime;			//!< feedback rupture time of the Haptic interface
	bool useForceFeedback;			//!< Flag stating if the force feedback is enabled in the teleoperation scheme
	Eigen::MatrixXf Rms;			//!< Rotation matrix expressing the orientation of the slave system (robot) wrt the haptic system (HapticInterface)

};


#endif // HAPTIC_INTERFACE_HPP_
