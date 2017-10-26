#include <math.h>
#include "type_methode.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
#include "commons.h"
#include "pid.h"
using namespace Eigen;

class Controller
{
public:
	Controller()
	: e_p()
	, e_v()
	, e_R()
	, e_gyro()
	, Zb_des()
	, Xc_des()
	, Xb_des()
	, Yb_des()
	, R_des()
	, m_pidX(
			40.0,
			20.0,
			2.0,
			2.0,
			0.6,
			-10.0,
			10.0,
			-0.1,
			0.1)
		,m_pidY(
			40.0,
			20.0,
			2.0,
			2.0,
			0.6,
			-10.0,
			10.0,
			0.1,
			0.1)
		,m_pidZ(
			5000.0,
			6000.0,
			3500.0,
			2.0,
			0.5,
			10000.0,
			60000.0,
			-1000.0,
			1000.0)
		,m_pidVx(
			5000.0,
			6000.0,
			3500.0,
			2.0,
			0.5,
			10000.0,
			60000.0,
			-1000.0,
			1000.0)
		,m_pidVy(
			5000.0,
			6000.0,
			3500.0,
			2.0,
			0.5,
			10000.0,
			60000.0,
			-1000.0,
			1000.0)
		,m_pidVz(
			5000.0,
			6000.0,
			3500.0,
			2.0,
			0.5,
			10000.0,
			60000.0,
			-1000.0,
			1000.0)
		/*,m_pidR(
			200.0,
			20.0,
			0.0,
			2.0,
			0.6,
			-200.0,
			200.0,
			0.0,
			0.0)
		,m_pidP(
			200.0,
			20.0,
			0.0,
			2.0,
			0.6,
			-200.0,
			200.0,
			0.0,
			0.0)
		,m_pidY(
			200.0,
			20.0,
			0.0,
			2.0,
			0.6,
			-200.0,
			200.0,
			0.0,
			0.0)*/
	{};
	const float K_p = 0.0f;
	const float K_v = 0.0f;
	const float K_R = 0.0f;
	const float K_RInt = 0.0f;
	const float K_gyro = 0.0f;
	const float K_gyroInt = 0.0f;
	const float w_Vicon = 0.85f;
	const float w_IMU = 0.15f;
	const float wv_Vicon = 0.85f;
	const float wv_IMU = 0.15f;
protected:
	//att:
	Vector3f e_p, e_v, e_R, e_R_Int, e_gyro, e_gyro_Int;
	Vector3f Zb_des, Xc_des, Yb_des, Xb_des, F_des;
	Vector3f l_gyro;
	Vector3f RPY_sp;
	Matrix3f R_des;
	Vector4f Q;
	/*PID m_pidR;
	PID m_pidP;
	PID m_pidYaw;*/
	//pos:
	Vector3f l_posVicon, acc_Sp, pos_Sp, pos_estIMU, lacc_est_IMU, l_posSp;
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	PID m_pidVx;
	PID m_pidVy;
	PID m_pidVz;

	void reset_attController(Vector3f* gyro){
		l_gyro = *gyro;
		e_gyro_Int.setZero();
		e_R_Int.setZero();
		R_des.setZero();
		RPY_sp.setZero();
	}

	void resetposController(Vector3f* pos_est_Vicon)
	{
		l_posVicon = *pos_est_Vicon;
	}
	void resetposSp(Vector3f* posSp)
	{
		l_posSp = *posSp;
	}
	void resetaccController(Vector3f* acc_est_IMU)
	{
		lacc_est_IMU = *acc_est_IMU;
	}
	void position_control(Vector3f* pos_est_Vicon, Vector3f* Sp, Vector4f* Output, Vector3f* acc_est_IMU, Matrix3f* R_est, float dt)
	{	
		Vector3f vel_Sp, vel_estVicon, vel_estIMU, acc_IMU_wd;
		body2earth(R_est, acc_est_IMU, &acc_IMU_wd, 3);
		for(int i=0; i<3; i++){
			pos_Sp(i) = (*Sp)(i);
		}
		vel_estIMU += acc_IMU_wd*dt;
		pos_estIMU += vel_estIMU*dt;

		float x_United = (*pos_est_Vicon)(0)*w_Vicon + pos_estIMU(0)*w_IMU;
		float y_United = (*pos_est_Vicon)(1)*w_Vicon + pos_estIMU(1)*w_IMU;
		float z_United = (*pos_est_Vicon)(2)*w_Vicon + pos_estIMU(2)*w_IMU;
		float x_sp = pos_Sp(0);
		float y_sp = pos_Sp(1);
		float z_sp = pos_Sp(2);

		//pos_estUnited = (*pos_est_Vicon)*w_Vicon + pos_estIMU*w_IMU;
		pos_Sp(0) = m_pidX.pid_update(x_United , x_sp , dt);
		pos_Sp(1) = m_pidY.pid_update(y_United , y_sp , dt);
		pos_Sp(2) = m_pidZ.pid_update(z_United , z_sp , dt);
		/*pos_Sp(0) = m_pidX.pid_update(pos_est_Vicon(0)*w_Vicon+pos_estIMU(0)*w_IMU , pos_Sp(0) , dt);
		pos_Sp(1) = m_pidY.pid_update(pos_est_Vicon(1)*w_Vicon+pos_estIMU(1)*w_IMU , pos_Sp(1) , dt);
		pos_Sp(2) = m_pidZ.pid_update(pos_est_Vicon(2)*w_Vicon+pos_estIMU(2)*w_IMU , pos_Sp(2) , dt);*/
		
		vec3f_integration(&vel_Sp, &pos_Sp, dt);
		vec3f_derivative(&vel_estVicon, pos_est_Vicon, &l_posVicon, dt);

		/*Vector3f vel_estUnited;
		vel_estUnited = vel_estVicon*wv_Vicon + vel_estIMU*wv_IMU;*/
		x_United = vel_estVicon(0)*wv_Vicon + vel_estIMU(0)*wv_IMU;
		y_United = vel_estVicon(1)*wv_Vicon + vel_estIMU(1)*wv_IMU;
		z_United = vel_estVicon(2)*wv_Vicon + vel_estIMU(2)*wv_IMU;
		x_sp = vel_Sp(0);
		y_sp = vel_Sp(1);
		z_sp = vel_Sp(2);

		vel_Sp(0) = m_pidVx.pid_update(x_United, x_sp, dt);
		vel_Sp(1) = m_pidVx.pid_update(y_United, y_sp, dt);
		vel_Sp(2) = m_pidVx.pid_update(z_United, z_sp, dt);
		
		vec3f_integration(&acc_Sp, &vel_Sp, dt);
		float thrust_force = acc_Sp.norm() * (float)VEHICLE_MASS / 2500.0;
		(*Output)(3) = thrust_force;
	}

	void attitude_control(Vector3f* Sp, Vector4f* Output, Matrix3f* R_est, Vector3f* gyro, float dt)
	{	
		F_des = (*Output)(3)*R_des.col(2);
		vec3f_passnorm(&F_des, &Zb_des);
		/*for(int i=0; i<3; i++)
			RPY_sp(i) = (*Sp)(i);*/
		for (int i=0; i<3; i++)
			R_des(i,2) = Zb_des(i);

		Xc_des(0) = cos((*Sp)(3));
		Xc_des(1) = sin((*Sp)(3));
		Xc_des(2) = 0;

		vec3f_cross(&Zb_des,&Xc_des,&Yb_des);
		vec3f_normalize(&Yb_des);
		vec3f_cross(&Yb_des, &Zb_des, &Xb_des);
		for (int i=0; i<3; i++)
			{
				R_des(i,0) = Xb_des(i);
				R_des(i,1) = Yb_des(i);
			}
		evv_map(&R_des, R_est, &e_R);

		e_gyro = *gyro - l_gyro;
		e_R_Int += e_R*dt;
		e_gyro_Int += e_gyro*dt;

		l_gyro = *gyro;

		*gyro += e_R*K_R + e_gyro*K_gyro + e_R_Int*K_RInt + e_gyro_Int*K_gyroInt;

		Matrix4f M_Q2Qdot;
  		//M_Q2Qdot.resize(4,4);
  		M_Q2Qdot << 0,      		-(*gyro)(0),  -(*gyro)(1),   -(*gyro)(2),
  		            (*gyro)(0),         0,         (*gyro)(2),   -(*gyro)(1),
  		            (*gyro)(1),     -(*gyro)(2),      0,          (*gyro)(0),
  		            (*gyro)(2),      (*gyro)(1),  -(*gyro)(0),            0;
		M_Q2Qdot /= 2.0f;
		//update:

  		Vector4f q_dot = (M_Q2Qdot*Q);
  		Q += q_dot*dt;
  		quaternion2rotation(&Q,&R_des);
  		rotation2euler(&R_des,&RPY_sp);
		//vec3f_integration(&RPY_sp, gyro, dt);
		for(int i=0;i<3;i++)
			(*Output)(i) = RPY_sp(i);
	}

	// map erro from so(3) to R3:
	void evv_map(const Matrix3f* R_des, const Matrix3f* R_B, Vector3f* e_R)
	{	
		//Matrix3f R_desT = R_des->transpose().eval();
		Matrix3f e_Rtemp = (R_des->transpose().eval() * (*R_B) - R_B->transpose().eval() * (*R_des))/2.0f;
		for(int i=0; i<3; i++){
			(*e_R)(i) = sqrt(e_Rtemp(i,0)*e_Rtemp(i,0) + e_Rtemp(i,1)*e_Rtemp(i,1) + e_Rtemp(i,2)*e_Rtemp(i,2));
		}
	}
};