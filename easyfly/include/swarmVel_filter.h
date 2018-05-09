class Velocity_filter{
    friend class ViconTracker;  //he is my friend, he can use my private members
public:
	Velocity_filter(Vector3f data_input,Vector3f result, Vector3f result_buffer)
	:data_input(data_input)
	,result(result)
	,result_buffer(result_buffer)
	,sign_buffer_v(3,1)
	,alpha_v(3,0.0f)
	,counter_max_v(3,50)
	,threshold_v(3,0.01)
    ,counter_v(3,0)
	{
	}
	//Vector3f& result,result_buffer; //data container
    Vector3f data_input;
    Vector3f result;
    Vector3f result_buffer;
	std::vector<float> counter_max_v; //threshold for counters
	std::vector<int> counter_v; //counter
	std::vector<float> threshold_v; //threshold for difference between two steps
	std::vector<int> sign_buffer_v; // direction de changment du dernier temps 
	std::vector<float> alpha_v; //filter hyper-parameter	
private:
	void update(Vector3f input) 
	{
		data_input = input;
	}
	Vector3f  filter_vec3f()
	{
		for(int i=0;i<3;++i){  //go over the 3-dimension vector
			if(directionNOTchanged(i)){ //same direction
				counter_v[i]++;
				if(fabs(diff_data(i))>threshold_v[i]){
					counter_v[i] += 8;
				}
				if(counter_v[i]>=counter_max_v[i]){
					counter_v[i] = 0;
					alpha_v[i] += 0.001;
				}
			} //same direction
			else{ //different direction: reset to zeros
				counter_v[i] = 0;
				alpha_v[i] = 0.0001;
				continue;
			}
            //printf("----------------------------alpha: %f ---------------------------\n",alpha_v[i]);
            alpha_v[i] = std::min(alpha_v[i],0.01f);
		}// i
		for(int i=0;i<3;++i){
			//alpha_v[i] = 0.001; //确定大范围
			result(i) = (1-alpha_v[i]) * result_buffer(i) + alpha_v[i]* data_input(i); //renew data
			sign_buffer_v[i] = sign(diff_data(i)); //renew sign buffer 
		}
		result_buffer = result; //renew result data
		return result;
	}
	bool directionNOTchanged(int dimension)
	{
		/*printf("---------sign: %f -------------\n",sign(diff_data(dimension)*sign_buffer_v[dimension]));
		printf("---------direction not changed: %d ---------------\n",sign(diff_data(dimension)*sign_buffer_v[dimension]) == 1.0f); */
		return sign(diff_data(dimension)*sign_buffer_v[dimension]) == 1.0f;
	}
	float sign(float data)
	{
		if (data>0||data==0)return 1.0f;
		else return -1.0f;
	}
	float diff_data(int dimension)
	{
		return 	data_input(dimension)-result_buffer(dimension);
	}
	std::vector<float> getAlpha_v()
	{
		return alpha_v;
	}
};