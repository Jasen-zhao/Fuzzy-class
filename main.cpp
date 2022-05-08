#include <iostream>
using namespace std;
#include "fuzzy_controller.h"
#include "_fuzzy.h"

void test2() {
	float v = 0;
	float d = 0;
	float u = 0;
	Fuzzy_controller fuzzy(35, 1, 1);//emax,demax,umax
	fuzzy.default_set();
	for (int i = 0; i < 10; i++)
	{
		cout << "请输入体积" << endl;
		cin >> v;
		cout << "请输入密度" << endl;
		cin >> d;
		u = fuzzy.realize(v, d);//u是分类的结果
		cout << i << "  " << v << "   " << d << "   " << u << endl;
	}
}

int main()
{
	//test1();//自定义测试
	test2();//默认测试
	
	float s = getData(20.0f, 0.01f);
	cout << s<< endl;
	system("pause");
	return 0;
}
