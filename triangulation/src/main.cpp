#include "triangulation.h"

int main()
{
	Triangulator tri;
	tri.set_quiet(true);
	tri.set_maximun_area(0.2);
	tri.set_minimun_angle(30);
	tri.testTriPoly();
	tri.testTriPolyWithHoles();
	cout << "cmd = " << tri.get_cmd() << endl;
	//int a; std::cin >> a;
	return 0;
} 