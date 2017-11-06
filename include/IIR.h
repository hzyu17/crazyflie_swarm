class IIR_I  
{ 
private:  
    float *m_pNum;  
    float *m_pDen; 
    float *m_px; 
    float *m_py; 
    int m_num_order;  
    int m_den_order; 
    
public: 
    /*float *m_O2a_params;
    float *m_O2b_params;*/
    IIR_I();  
    void reset();  
    void setPara(float num[], int num_order, float den[], int den_order);  
    void resp(float data_in[], int m, float data_out[], int n); 
    float filter(float data); 
    void filter(float data[], int len);  
    void filter(float data_in[], float ata_out[], int len);  
    void calculO2param(float f0, float fs, float db, float Q, float As[], float Bs[]);
}; 