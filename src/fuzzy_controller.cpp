#include "fuzzy_controller.h"
using namespace std;
//试试

Fuzzy_controller::Fuzzy_controller(float e_max, float de_max, float u_max) :
	vmax(e_max), dmax(de_max), umax(u_max), v_mf_paras(NULL), d_mf_paras(NULL), u_mf_paras(NULL)
{
	Kv = 2 * (N / 2) / vmax;
	Kd = 2 * (N / 2) / dmax;
	mf_t_v = "trimf";
	mf_t_d = "trimf";
	mf_t_u = "trimf";
}

Fuzzy_controller::~Fuzzy_controller()
{
	delete[] v_mf_paras;
	delete[] d_mf_paras;
	delete[] u_mf_paras;
}
//三角隶属度函数
float Fuzzy_controller::trimf(float x, float a, float b, float c)
{
	float u;
	if (x >= a && x <= b)
		u = (x - a) / (b - a);
	else if (x > b && x <= c)
		u = (c - x) / (c - b);
	else
		u = 0.0;
	return u;

}

//设置模糊规则
void Fuzzy_controller::setRule(int rulelist[N][N])
{
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			rule[i][j] = rulelist[i][j];
}

//设置模糊隶属度函数的类型和参数
void Fuzzy_controller::setMf(const string& mf_type_e, float* e_mf, const string& mf_type_de, float* de_mf, const string& mf_type_u, float* u_mf)
{
	if (mf_type_e == "trimf" || mf_type_e == "gaussmf" || mf_type_e == "trapmf")
		mf_t_v = mf_type_e;
	else
		cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;

	if (mf_type_de == "trimf" || mf_type_de == "gaussmf" || mf_type_de == "trapmf")
		mf_t_d = mf_type_de;
	else
		cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;

	if (mf_type_u == "trimf" || mf_type_u == "gaussmf" || mf_type_u == "trapmf")
		mf_t_u = mf_type_u;
	else
		cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
	v_mf_paras = new float[N * 3];
	d_mf_paras = new float[N * 3];
	u_mf_paras = new float[N * 3];
	for (int i = 0; i < N * 3; i++)
		v_mf_paras[i] = e_mf[i];
	for (int i = 0; i < N * 3; i++)
		d_mf_paras[i] = de_mf[i];
	for (int i = 0; i < N * 3; i++)
		u_mf_paras[i] = u_mf[i];
}
//实现模糊控制
float Fuzzy_controller::realize(float v, float d)
{
	float u_v[N], u_d[N];
	int u_v_index[3], u_d_index[3];//假设一个输入最多激活3个模糊子集
	float u;
	int M;
	if (v == 0)	v = 0.00001f;
	if (d == 0)	d = 0.00001f;
	v = v - (vmax / 2);//把输入减去输入最大值的一半，
	d = d - (dmax / 2);
	v = Kv * v;
	d = Kd * d;
	if (mf_t_v == "trimf")
		M = 3;               //三角函数有三个参数
	else if (mf_t_v == "gaussmf")
		M = 2;              //正态函数有两个参数
	else if (mf_t_v == "trapmf")
		M = 4;              //梯形函数有四个参数
	int j = 0;
	for (int i = 0; i < N; i++)
	{
		u_v[i] = trimf(v, v_mf_paras[i * M], v_mf_paras[i * M + 1], v_mf_paras[i * M + 2]);//e模糊化，计算它的隶属度
		if (u_v[i] != 0)
			u_v_index[j++] = i;                                              //存储被激活的模糊子集的下标，可以减小计算量
	}
	for (; j < 3; j++)u_v_index[j] = 0;

	if (mf_t_d == "trimf")
		M = 3;              //三角函数有三个参数
	else if (mf_t_d == "gaussmf")
		M = 2;              //正态函数有两个参数
	else if (mf_t_d == "trapmf")
		M = 4;               //梯形函数有四个参数
	j = 0;
	for (int i = 0; i < N; i++)
	{
		u_d[i] = trimf(d, d_mf_paras[i * M], d_mf_paras[i * M + 1], d_mf_paras[i * M + 2]);//de模糊化，计算它的隶属度
		if (u_d[i] != 0)
			u_d_index[j++] = i;                                                    //存储被激活的模糊子集的下标，可以减小计算量
	}
	for (; j < 3; j++)u_d_index[j] = 0;

	float den = 0, num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_v[u_v_index[m]] * u_d[u_d_index[n]] * rule[u_v_index[m]][u_d_index[n]];
			den += u_v[u_v_index[m]] * u_d[u_d_index[n]];
		}
	u = num / den;//u是隶属度，靠近谁就隶属于谁
	if (u >= umax - 0.01f)   u = umax;
	else if (u <= -umax)  u = -umax;
	return u;
}

void Fuzzy_controller::default_set() {
	int ruleMatrix[3][3] = { {NC,MC,MC},{NC,PC,PC},{NC,PC,PC} };//行是密度，列是体积
	float v_mf_paras[9] = { -1,-1,-0.76,-0.84,-0.52,1,1,1,1 };
	float d_mf_paras[9] = { -1,-1,-0.79999,-0.8,-0.6,0.4,0,1,1 };
	float u_mf_paras[9] = { -1,-1,-0.399999999,-0.4,0,0.3999999999,0.4,1,1 };
	this->setMf("trimf", v_mf_paras, "trimf", d_mf_paras, "trimf", u_mf_paras);
	this->setRule(ruleMatrix);
}



