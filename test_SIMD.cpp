#include <math.h>
#include <arm_neon.h>
#include <iostream>

using namespace std;

int main() {
	
	cout << "Test Code" << "\n";

	cout << "Test Vector Storing and Retrieving" << "\n";
	float testArray[4] = {10, 20, 30, 40};
	float32x4_t se_Vector = vld1q_f32(testArray);
	float retrieving[4];
	vst1q_f32(retrieving, se_Vector);
	cout << retrieving[0] << endl;
	cout << retrieving[1] << endl;
	cout << retrieving[2] << endl;
	cout << retrieving[3] << endl;

	cout << "Testing Divide" << endl;
	float testDivider[4] = {0.1, 0.1, 0.1, 0.1};
	float32x4_t div = vld1q_f32(testDivider);
	se_Vector = vmulq_f32(se_Vector, div);
	vst1q_f32(retrieving, se_Vector);
	cout << retrieving[0] << endl;
	cout << retrieving[1] << endl;
	cout << retrieving[2] << endl;
	cout << retrieving[3] << endl;

	float testSqrts[4] = {4.0f, 16.0f, 25.0f, 36.0f};
	float32x4_t sqrts = vresqrteq_f32(vld1q_f32(testSqrts));
	vst1q_f32(retrieving, sqrts);
	cout << retrieving[0] << endl;
	cout << retrieving[1] << endl;
	cout << retrieving[2] << endl;
	cout << retrieving[3] << endl;

	return 0;
}
