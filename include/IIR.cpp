#include "/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include/IIR.h"
#include <math.h>
#include <iostream>

#define _USE_MATH_DEFINES //PI
 void IIR_I::reset()  
{  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_pNum[i] = 0.0;  
    }  
    for(int i = 0; i <= m_den_order; i++)  
    {  
        m_pDen[i] = 0.0;  
    }  
}  
IIR_I::IIR_I()  
{  
    m_pNum = nullptr;  
    m_pDen = nullptr;  
    m_px = nullptr;  
    m_py = nullptr;  
    m_num_order = -1;  
    m_den_order = -1;  
};  
/** \brief 
 * 
 * \param num 分子多项式的系数，升序排列,num[0] 为常数项 
 * \param m 分子多项式的阶数 
 * \param den 分母多项式的系数，升序排列,den[0] 为常数项 
 * \param m 分母多项式的阶数 
 * \return 
 */  
void IIR_I::setPara(float num[], int num_order, float den[], int den_order)  
{  
    delete[] m_pNum;  
    delete[] m_pDen;  
    delete[] m_px;  
    delete[] m_py;  
    m_pNum = new float[num_order + 1];  
    m_pDen = new float[den_order + 1];  
    m_num_order = num_order;  
    m_den_order = den_order;  
    m_px = new float[num_order + 1];  
    m_py = new float[den_order + 1];  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_pNum[i] = num[i];  
        m_px[i] = 0.0;  
    }  
    for(int i = 0; i <= m_den_order; i++)  
    {  
        m_pDen[i] = den[i];  
        m_py[i] = 0.0;  
    }  
}  
  
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data 传入输入数据 
 * \return 滤波后的结果 
 */  
float IIR_I::filter(float data)  
{  
    m_py[0] = 0.0; // 存放滤波后的结果  
    m_px[0] = data;  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_py[0] = m_py[0] + m_pNum[i] * m_px[i];  
    }  
    for(int i = 1; i <= m_den_order; i++)  
    {  
        m_py[0] = m_py[0] - m_pDen[i] * m_py[i];  
    }  
    for(int i = m_num_order; i >= 1; i--)  
    {  
        m_px[i] = m_px[i-1];  
    }  
    for(int i = m_den_order; i >= 1; i--)  
    {  
        m_py[i] = m_py[i-1];  
    }  
    return m_py[0];  
}  
  
  
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data[] 传入输入数据，返回时给出滤波后的结果 
 * \param len data[] 数组的长度 
 * \return 
 */  
void IIR_I::filter(float data[], int len)  
{  
    int i, k;  
    for(k = 0; k < len; k++)  
    {  
        m_px[0] = data[k];  
        data[k] = 0.0;  
        for(i = 0; i <= m_num_order; i++)  
        {  
            data[k] = data[k] + m_pNum[i] * m_px[i];  
        }  
        for(i = 1; i <= m_den_order; i++)  
        {  
            data[k] = data[k] - m_pDen[i] * m_py[i];  
        }  
        // we get the y value now  
        m_py[0] = data[k];  
        for(i = m_num_order; i >= 1; i--)  
        {  
            m_px[i] = m_px[i-1];  
        }  
        for(i = m_den_order; i >= 1; i--)  
        {  
            m_py[i] = m_py[i-1];  
        }  
    }  
}  
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data_in[] 输入数据 
 * \param data_out[] 保存滤波后的数据 
 * \param len 数组的长度 
 * \return 
 */  
void IIR_I::filter(float data_in[], float data_out[], int len)  
{  
    int i, k;  
    for(k = 0; k < len; k++)  
    {  
        m_px[0] = data_in[k];  
        m_py[0] = 0.0;  
        for(i = 0; i <= m_num_order; i++)  
        {  
            m_py[0] = m_py[0] + m_pNum[i] * m_px[i];  
        }  
        for(i = 1; i <= m_den_order; i++)  
        {  
            m_py[0] = m_py[0] - m_pDen[i] * m_py[i];  
        }  
        for(i = m_num_order; i >= 1; i--)  
        {  
            m_px[i] = m_px[i-1];  
        }  
        for(i = m_den_order; i >= 1; i--)  
        {  
            m_py[i] = m_py[i-1];  
        }  
        data_out[k] = m_py[0];  
    }  
}  


    /** \brief 计算 IIR 滤波器的时域响应，不影响滤波器的内部状态 
     * \param data_in 为滤波器的输入，0 时刻之前的输入默认为 0，data_in[M] 及之后的输入默认为data_in[M-1] 
     * \param data_out 滤波器的输出 
     * \param M 输入数据的长度 
     * \param N 输出数据的长度 
     * \return 
     */  
    void IIR_I::resp(float data_in[], int M, float data_out[], int N)  
    {  
        int i, k, il;  
        for(k = 0; k < N; k++)  
        {  
            data_out[k] = 0.0;  
            for(i = 0; i <= m_num_order; i++)  
            {  
                if( k - i >= 0)  
                {  
                    il = ((k - i) < M) ? (k - i) : (M - 1);  
                    data_out[k] = data_out[k] + m_pNum[i] * data_in[il];  
                }  
            }  
            for(i = 1; i <= m_den_order; i++)  
            {  
                if( k - i >= 0)  
                {  
                    data_out[k] = data_out[k] - m_pDen[i] * data_out[k - i];  
                }  
            }  
        }  
    }  
    void IIR_I::calculO2param(float f0, float fs, float db, float Q, float As[], float Bs[])
    {
    	
    	float Omega = 2*M_PI*f0/fs;
    	float s = sin(Omega);
    	float c = cos(Omega);
    	float alpha = sin(2*Q);
    	As[0] = 1.0f;
    	As[1] = -2*c/(1 + alpha);
    	As[2] = (1 - alpha)/(1 + alpha);
    	Bs[0] = (1-c)/2.0;
    	Bs[1] = 1-c;
    	Bs[2] = (1-c)/2.0;
    }