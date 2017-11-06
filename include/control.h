#include <math.h>
#include "type_methode.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
#include "commons.h"
#include "pid.h"
#include "IIR.h"
using namespace Eigen;

class Controller
{
public:
	Controller()
	: loop_record(0)
	, m_group_index(0)
	, m_resFnameRoot("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/vehicle0/")
	, m_O2a_params()
	, m_O2b_params()
		,m_pidX(
			1.0,
			0.0,
			0.2,
			0.2,
			0.6,
			-1.0,
			1.0,
			-0.1,
			0.1)
		,m_pidY(
			1.0,
			0.0,
			0.2,
			0.2,
			0.6,
			-1.0,
			1.0,
			0.1,
			0.1)
		,m_pidZ(
			1.0,
			0.0,
			0.2,
			0.2,
			0.6,
			-1.0,
			1.0,
			0.1,
			0.1)
		,m_pidVx(
			0.6,
			0.0,
			0.4,
			0.2,
			0.5,
			0.1,
			1.0,
			-5.0,
			5.0)
		,m_pidVy(
			0.6,
			0.0,
			0.4,
			0.2,
			0.5,
			0.1,
			1.0,
			-5.0,
			5.0)
		,m_pidVz(
			1.5,
			0.1,
			1.0,
			0.2,
			0.5,
			0.1,
			3.0,
			-5.0,
			5.0)
	{vel_estIMU.setZero();
	 vel_Sp.setZero();
	 vel_estVicon.setZero();
	 acc_IMU_wd.setZero();
	 acc_Sp.setZero();
	 e_p.setZero();
	 e_v.setZero();
	 e_R.setZero();
	 e_gyro.setZero();
	 Zb_des.setZero();
	 Xc_des.setZero();
	 Xb_des.setZero();
	 Yb_des.setZero();
	 R_des.setZero();
	 q_dot.setZero();
	 M_Q2Qdot.setZero();
	 err_pos.setZero();
	 err_vel.setZero();
	 pos_Sp.setZero();
	 pos_estIMU.setZero();
	 R_est.setZero();
	 l_possp.setZero();
	 l_velsp.setZero();
	 m_pidX.reset();
	 m_pidY.reset();
	 m_pidZ.reset();
	 m_pidVx.reset();
	 m_pidVy.reset();
	 m_pidVz.reset();
	 m_acc_IIR.reset();
	 
	};
	const float K_p = 0.8f;
	const float K_v = 0.8f;
	const float K_R = 1.0f;
	const float K_RInt = 0.8f;
	const float K_gyro = 0.8f;
	const float K_gyroInt = 0.8f;
	const float w_Vicon = 0.9f;
	const float w_IMU = 0.1f;
	const float wv_Vicon = 0.9f;
	const float wv_IMU = 0.1f;
	const float m_f0 = 4.0f; //cutoff f
    const float m_fs = 1000.0f; //sampling f
    const float m_gain_db = 0.0f; //gain
    const float m_Q_filter = 1.0f; //quality
	char m_resFnameRoot[150];
	int loop_record;
	int m_group_index;
protected:
	//att:
	Vector3f e_p, e_v, e_R, e_R_Int, e_gyro, e_gyro_Int;
	Vector3f Zb_des, Xc_des, Yb_des, Xb_des, Thrust_des;//, F_des
	Vector3f l_gyro;
	Vector3f RPY_sp;
	Matrix3f R_des, R_est;
	Vector4f Q, q_dot, l_posSp_whole;
	Matrix4f M_Q2Qdot;
	Vector3f vel_Sp, vel_estVicon, vel_estIMU, acc_IMU_wd;
	Vector3f err_pos, err_vel;
	IIR_I m_acc_IIR;
	/*PID m_pidR;
	PID m_pidP;
	PID m_pidYaw;*/
	//pos:
	Vector3f l_posVicon, acc_Sp, pos_Sp, l_possp, pos_estIMU, lacc_est_IMU, l_velsp;
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	PID m_pidVx;
	PID m_pidVy;
	PID m_pidVz;
	float m_O2a_params[2];
    float m_O2b_params[2];

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
		l_possp = *pos_est_Vicon;
	}
	void resetposSp(Vector4f* posSp)
	{
		l_posSp_whole = *posSp;
	}
	void resetaccController(Vector3f* acc_est_IMU)
	{
		lacc_est_IMU = *acc_est_IMU;
	}
	void position_control(Vector3f* pos_est_Vicon, Vector4f* Sp, Vector4f* Output, Vector3f* acc_est_IMU, Vector3f* Euler, float dt)
	{	
		if(dt>1.0f)
		{}
		else{
			
			euler2rotation(Euler,&R_est);
			body2earth(&R_est, acc_est_IMU, &acc_IMU_wd, 3);
			for(int i=0; i<3; i++){
				pos_Sp(i) = (*Sp)(i);
			}
			//vel_estIMU.setZero();
			//m_acc_IIR.calculO2param(m_f0, m_fs, m_gain_db, m_Q_filter,m_O2a_params,m_O2b_params); //f0,fs,db,Q
			m_O2a_params[0] = 1.0f;
			m_O2a_params[1] = -1.80898117793047f;
			m_O2a_params[2] = 0.827224480562408f;
			m_O2b_params[0] = 0.095465967120306f;
			m_O2b_params[1] = -0.172688631608676f;
			m_O2b_params[2] = 0.095465967120306f;
			m_acc_IIR.setPara(m_O2b_params,2,m_O2a_params,2);
			//printf("As:  %f   %f   %f\n",m_O2a_params[0],m_O2a_params[1],m_O2a_params[2] );
			//printf("Bs:  %f   %f   %f\n",m_O2b_params[0],m_O2b_params[1],m_O2b_params[2] );
			acc_IMU_wd(0) = m_acc_IIR.filter(acc_IMU_wd(0));
			acc_IMU_wd(1) = m_acc_IIR.filter(acc_IMU_wd(1));
			acc_IMU_wd(2) = m_acc_IIR.filter(acc_IMU_wd(2));
	 		/*m_O2a_params = m_acc_IIR.m_O2a_params;
	 		m_O2b_params = m_acc_IIR.m_O2b_params;*/
	 		
			
			acc_IMU_wd(2) = acc_IMU_wd(2);
			printf("ACC %f   %f   %f\n", acc_IMU_wd(0),acc_IMU_wd(1),acc_IMU_wd(2));
			vel_estIMU += acc_IMU_wd*dt; //tested: good
			/*printf("accIMUwd: %f %f  %f\n",acc_IMU_wd(0),acc_IMU_wd(1),acc_IMU_wd(2) );
			printf("vel_estIMU:  %f   %f   %f\n", vel_estIMU(0), vel_estIMU(1), vel_estIMU(2));*/
			/*float x_United = (*pos_est_Vicon)(0);//*w_Vicon + pos_estIMU(0)*w_IMU;
			float y_United = (*pos_est_Vicon)(1);//*w_Vicon + pos_estIMU(1)*w_IMU;
			float z_United = (*pos_est_Vicon)(2);//*w_Vicon + pos_estIMU(2)*w_IMU;
	
			float x_sp = pos_Sp(0);
			float y_sp = pos_Sp(1);
			float z_sp = pos_Sp(2);*/
			//pos_estUnited = (*pos_est_Vicon)*w_Vicon + pos_estIMU*w_IMU;
			/*pos_Sp(0) = m_pidX.pid_update(x_United , x_sp , dt);
			pos_Sp(1) = m_pidY.pid_update(y_United , y_sp , dt);
			pos_Sp(2) = m_pidZ.pid_update(z_United , z_sp , dt);*/
			
			//printf("POS-sp:  %f   %f   %f\n", pos_Sp(0), pos_Sp(1), pos_Sp(2));
			/*printf("l_possp x:  %f\n",l_possp(0));
			printf("pos_Sp x:   %f\n",pos_Sp(0) );
			printf("dt:  %f\n",dt);
			float vv = (pos_Sp(0) - l_possp(0))/dt;
			printf("vv:  %f \n",vv );*/
			vec3f_derivative(&vel_Sp, &pos_Sp, &l_possp, dt);
			vec3f_derivative(&vel_estVicon, pos_est_Vicon, &l_posVicon, dt);
			
			//printf("posSP: %f  %f  %f\nposVicon: %f  %f  %f\n",pos_Sp(0),pos_Sp(1),pos_Sp(2),(*pos_est_Vicon)(0),(*pos_est_Vicon)(1),(*pos_est_Vicon)(2) );
			l_posVicon = *pos_est_Vicon;
			l_possp = pos_Sp;
			
	
			float x_United = vel_estVicon(0)*wv_Vicon + vel_estIMU(0)*wv_IMU;
			float y_United = vel_estVicon(1)*wv_Vicon + vel_estIMU(1)*wv_IMU;
			float z_United = vel_estVicon(2)*wv_Vicon + vel_estIMU(2)*wv_IMU;
	
			float x_sp = vel_Sp(0);
			float y_sp = vel_Sp(1);
			float z_sp = vel_Sp(2);
			
			//printf("VEL_SP:  %f     %f     %f\n", x_sp, y_sp, z_sp);
			//printf("DIFFERENCES VEL:  %f    %f    %f\n",x_sp-x_United,y_sp-y_United,z_sp-z_United);
			vel_Sp(0) = m_pidVx.pid_update(x_United, x_sp, dt);
			vel_Sp(1) = m_pidVy.pid_update(y_United, y_sp, dt);
			vel_Sp(2) = m_pidVz.pid_update(z_United, z_sp, dt);
			

			err_vel.setZero();
			err_pos.setZero();
			err_pos = pos_Sp - (*pos_est_Vicon);// + pos_estIMU*w_IMU);
			err_vel = vel_Sp - (vel_estVicon*wv_Vicon + vel_estIMU*wv_IMU);
			//printf("VEL-sp:  %f   %f   %f\n %f   %f   %f\n", vel_Sp(0), vel_Sp(1), vel_Sp(2),l_velsp(0),l_velsp(1),l_velsp(2));
			vec3f_derivative(&acc_Sp, &vel_Sp, &l_velsp, dt);
			l_velsp = vel_Sp;
			//printf("ACC-sp:  %f   %f   %f\n", acc_Sp(0), acc_Sp(1), acc_Sp(2));
			Thrust_des = acc_Sp * (float)VEHICLE_MASS;// + K_p*err_pos + K_v*err_vel;
			//Thrust_des(2) += GRAVITY/1000.0f * (float)VEHICLE_MASS;
			float thrust_force = Thrust_des.norm()/1000.0f; // * (float)VEHICLE_MASS / 2500.0;
			//printf("thrust_force:  %f\n", thrust_force);
			(*Output)(3) = thrust_force;
			if(loop_record<=6000)
		{
			loop_record++;
			char fname[250];
			sprintf(fname,"%spos_est_Vicon.dat",m_resFnameRoot);
			writeData_bin(fname,pos_est_Vicon);
			sprintf(fname,"%sposSp.dat",m_resFnameRoot);
			writeData_bin(fname,&pos_Sp);
			sprintf(fname,"%svel_sp0.dat",m_resFnameRoot);
			writeData_bin(fname,&vel_Sp);
			sprintf(fname,"%saccSp.dat",m_resFnameRoot);
			writeData_bin(fname,&acc_Sp);
			sprintf(fname,"%sImuAcc_est.dat",m_resFnameRoot);
			writeData_bin(fname,acc_est_IMU);
			sprintf(fname,"%svel_estVicon.dat",m_resFnameRoot);
			writeData_bin(fname,&vel_estVicon);
			sprintf(fname,"%svel_estIMU.dat",m_resFnameRoot);
			writeData_bin(fname,&vel_estIMU);
			sprintf(fname,"%sImuAccWD_est.dat",m_resFnameRoot);
			writeData_bin(fname,&acc_IMU_wd);
		}
		}
		

	}

	void attitude_control(Vector4f* Sp, Vector4f* Output, Vector3f* Euler, Vector3f* gyro, float dt)
	{	
		e_gyro.setZero();
		e_R.setZero();
		if(dt>1.0f)
		{}
		else{
			euler2rotation(Euler,&R_est);
			//F_des = (*Output)(3)*R_est.col(2);
			Thrust_des = Thrust_des/100000.0f;
			//printf("Thrust_des:  %f   %f   %f\n", Thrust_des(0),Thrust_des(1),Thrust_des(2));
			vec3f_passnorm(&Thrust_des, &Zb_des);
			//printf("Zb_des: %f  %f  %f\n", Zb_des(0), Zb_des(1), Zb_des(2));
			/*for(int i=0; i<3; i++)
				RPY_sp(i) = (*Sp)(i);*/
			for (int i=0; i<3; i++)
				R_des(i,2) = Zb_des(i);
	
			Xc_des(0) = cos((*Sp)(3));
			Xc_des(1) = sin((*Sp)(3));
			Xc_des(2) = 0;
			//printf("Sp(3):  %f\n", (*Sp)(3));
			vec3f_cross(&Zb_des, &Xc_des, &Yb_des);
			vec3f_normalize(&Yb_des);
			vec3f_cross(&Yb_des, &Zb_des, &Xb_des);
			for (int i=0; i<3; i++)
				{
					R_des(i,0) = Xb_des(i);
					R_des(i,1) = Yb_des(i);
				}
	
			evv_map(&R_des, &R_est, &e_R);
	
			e_gyro = *gyro - l_gyro;
			e_R_Int += e_R*dt;
			e_gyro_Int += e_gyro*dt;
	
			l_gyro = *gyro;
	
			*gyro = *gyro - e_R*K_R + e_gyro*K_gyro - e_R_Int*K_RInt + e_gyro_Int*K_gyroInt;
	
  			M_Q2Qdot.setZero();
  			M_Q2Qdot << 0,-(*gyro)(0),-(*gyro)(1),-(*gyro)(2),
  			            (*gyro)(0),0,(*gyro)(2),-(*gyro)(1),
  			            (*gyro)(1),-(*gyro)(2),0,(*gyro)(0),
  			            (*gyro)(2),(*gyro)(1),-(*gyro)(0),0;
			M_Q2Qdot /= 2.0f;  //minus for the vicon
			//update:
	
  			q_dot = M_Q2Qdot*Q;
  			Q += q_dot*dt;
  			quaternion2rotation(&Q,&R_des);
  			rotation2euler(&R_des,&RPY_sp);
  			euler2rotation(&RPY_sp,&R_des); //for the vicon reference differences
  			//printf("EULER SPS: %f   %f   %f\n",RPY_sp(0),RPY_sp(1),RPY_sp(2));
			//vec3f_integration(&RPY_sp, gyro, dt);
			for(int i=0;i<3;i++)
				(*Output)(i) = RPY_sp(i);
		}
		
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