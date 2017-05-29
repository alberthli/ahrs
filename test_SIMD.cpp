#include <math.h>
#include <arm_neon.h>
#include <iostream>
using namespace std;

int main() {
	
	cout << "Test Code" << "\n";
	cout << "Test Vector Storing and Retreaving" << "\n";
	
	float32x4_t se_Vector =ld1q_f32([10, 20, 30, 40]);
	float[4] retreaving;
	vst1q_f32(retreaving, se_Vector);
	cout << retreaving[0] << endl;
	cout << retreaving[1] << endl;
	cout << retreaving[2] << endl;
	cout << retreaving[3] << endl;


	cout << "Testing Divide" << endl;
	float32x4_t se_Vector =ld1q_f32([0.1, 0.1, 0.1, 0.1]);
	se_Vector = vmulq_f32(se_Vector, float32x4_t b);
	vst1q_f32(retreaving, se_Vector);
	cout << retreaving[0] << endl;
	cout << retreaving[1] << endl;
	cout << retreaving[2] << endl;
	cout << retreaving[3] << endl;

	return 0;

}
