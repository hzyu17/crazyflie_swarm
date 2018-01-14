#include <math.h>
#include "type_methode.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
using namespace Eigen;

class Attitude_estimator 
{
public:
	Attitude_estimator()
	: m_init_North()
	, m_erroM()
	{
		const float K_Pa = 0.3f;
		const float K_Ia = 0.1f;
		const float K_Pm = 0.2f;
		const float K_Im = 0.1f;
		m_Kcoefs(0) = K_Pa;
  	 	m_Kcoefs(1) = K_Ia;
  	 	m_Kcoefs(2) = K_Pm;
  	 	m_Kcoefs(3) = K_Im; 
  	 	m_erroM.resize(3,4);
	};
	/*Vector4f m_Kcoefs;
	Vector3f m_init_North;
	MatrixXf m_erroM;*/
protected:
	Vector3f m_init_North;
	Vector4f m_Kcoefs;
	MatrixXf m_erroM;

	void attitude_reset(const Vector3f* acc, const Vector3f* mag, Att* att)
{
  	Vector3f k, I, J;
  	float mag_dot_k;
  	printf("MAG:  %f    %f    %f\n",(*mag)(0),(*mag)(1),(*mag)(2));
  	k = -*acc;
  	vec3f_normalize(&k);
  	mag_dot_k = vec3f_dot(mag, &k);
  	I = *mag - mag_dot_k * k;
  	vec3f_normalize(&I);
  	vec3f_cross(&k, &I, &J);
  	for (int i=0; i<3; i++){
  	(att->R)(0,i) = I(i);
  	(att->R)(1,i) = J(i);
  	(att->R)(2,i) = k(i);
  	}
  	rotation2quaternion(&att->R, &att->Q);
  	rotation2euler(&att->R, &att->Euler);
  	//m_prevsT_integ_err = ros::Time::now();
  	 /*m_Kcoefs(0) = K_Pa;
  	 m_Kcoefs(1) = K_Ia;
  	 m_Kcoefs(2) = K_Pm;
  	 m_Kcoefs(3) = K_Im; */ 
}

	void  attitude_estimation(Att* att, Vector3f* acc, Vector3f* gyro, Vector3f* mag, float dt)
{
  	vec3f_normalize(acc);
  	vec3f_normalize(mag);

  	Vector3f err_a, err_g, R_lastRoll, err_a_Int;
  	Vector3f err_m_earth, mag_earth, err_m_Int;
  	Vector4f q_dot;
  	Matrix4f temp;
  	//MatrixXf (3,1);
  	R_lastRoll = (att->R).row(2);
	
  	vec3f_cross(acc, &R_lastRoll, &err_a);
  	body2earth(&(att->R),mag,&mag_earth,3);
  	vec3f_cross(&m_init_North, &mag_earth, &err_m_earth);
	
  	err_m_Int += err_m_earth*dt;
  	err_a_Int += err_a*dt;
	
	for(int i=0; i<3; i++)
	{
  	m_erroM(i,0) = err_a(i);
  	m_erroM(i,1) = err_a_Int(i);
  	m_erroM(i,2) = err_m_earth(i);
  	m_erroM(i,3) = err_m_Int(i);
  	}
  	(*gyro) += m_erroM * m_Kcoefs;
  	Matrix4f M_Q2Qdot;
  	M_Q2Qdot << 0,      		-(*gyro)(0),  -(*gyro)(1),   -(*gyro)(2),
  	            (*gyro)(0),         0,         (*gyro)(2),   -(*gyro)(1),
  	            (*gyro)(1),     -(*gyro)(2),      0,          (*gyro)(0),
  	            (*gyro)(2),      (*gyro)(1),  -(*gyro)(0),            0;
	M_Q2Qdot /= 2.0f;
	//update:
  	//printf("HELLO11111!!!\n");    
  	q_dot = (M_Q2Qdot*(att->Q));
  	att->Q += q_dot*dt;
  	quaternion2rotation(&(att->Q),&(att->R));
  	rotation2euler(&(att->R),&(att->Euler));
	}
};

