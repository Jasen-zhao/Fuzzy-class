#pragma once
#ifndef FUZZY_CONTROLLER_H_
#define FUZZY_CONTROLLER_H_
#include <iostream>
#define N 3
#define NC -1
#define MC 0
#define PC 1

class Fuzzy_controller
{
	//public:
	//	const static int N=7;//定义量化论域模糊子集的个数
private:
	float v;//输入变量，体积
	float d;//输入变量，密度
	float vmax;  //误差基本论域上限
	float dmax; //误差辩化率基本论域的上限
	float umax;  //输出的隶属度上限，这里是3
	float Kv;    //Kv=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	float Kd;   //Kd=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
	int rule[N][N];//模糊规则表
	std::string mf_t_v;   //e的隶属度函数类型
	std::string mf_t_d;  //de的隶属度函数类型
	std::string mf_t_u;   //u的隶属度函数类型
	float* v_mf_paras; //误差的隶属度函数的参数
	float* d_mf_paras;//误差的偏差隶属度函数的参数
	float* u_mf_paras; //输出的隶属度函数的参数

public:
	Fuzzy_controller(float v_max, float d_max, float u_max);
	~Fuzzy_controller();
	float trimf(float x, float a, float b, float c);          //三角隶属度函数
	//设置模糊隶属度函数的参数
	void setMf(const std::string& mf_type_v, float* v_mf, const std::string& mf_type_d, float* d_mf, const std::string& mf_type_u, float* u_mf);
	void setRule(int rulelist[N][N]);                          //设置模糊规则
	float realize(float t, float a);              //实现模糊控制
	void default_set();//默认设置，将模型初始化
};

#endif

