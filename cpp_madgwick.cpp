#include <math.h>
#include <arm_neon.h>

void madgwickFilterUpdate(){

		// Update Time

	//////////////////////////////wgewegweg egmwennned need to fix
		currTime = time.clock()
		float dt = currTime - prevTime
		prevTime = currTime
	//////////////////////////////afwegwegwegwegw this time stuff


		/*********************************/
		/* Useful Variable Manipulations */
		/*********************************/
		float32x4_t se_Vector = ld1q_f32([hSEq1, hSEq2, hSEq3, hSEq4]);

		float hSEq1 = 0.5 * SEq1
		float hSEq2 = 0.5 * SEq2
		float hSEq3 = 0.5 * SEq3
		float hSEq4 = 0.5 * SEq4

		float dSEq1 = 2 * SEq1
		float dSEq2 = 2 * SEq2
		float dSEq3 = 2 * SEq3
		float dSEq4 = 2 * SEq4

		float sSEq3 = SEq3 * SEq3

		float dbx = 2 * bx
		float dbz = 2 * bz

		float dbxSEq1 = dbx * SEq1
		float dbxSEq2 = dbx * SEq2
		float dbxSEq3 = dbx * SEq3
		float dbxSEq4 = dbx * SEq4
		float dbzSEq1 = dbz * SEq1
		float dbzSEq2 = dbz * SEq2
		float dbzSEq3 = dbz * SEq3
		float dbzSEq4 = dbz * SEq4

		float SEq1SEq3 = SEq1 * SEq3
		float SEq2SEq4 = SEq2 * SEq4

		/**************************/
		/* Beginning of Algorithm */
		/**************************/

		// Normalize acceleration and magnetometer values
		float tempNorm = sqrt(ax * ax + ay * ay + az * az)
		ax /= tempNorm
		ay /= tempNorm
		az /= tempNorm

		tempNorm = sqrt(mx * mx + my * my + mz * mz)
		mx /= tempNorm
		my /= tempNorm
		mz /= tempNorm

		float dmx = 2 * mx
		float dmy = 2 * my
		float dmz = 2 * mz

		// Combined cost function + Jacobian
		// Functions from g-field
		float f1 = dSEq2 * SEq4 - dSEq1 * SEq3 - ax
		float f2 = dSEq1 * SEq2 + dSEq3 * SEq4 - ay
		float f3 = 1 - dSEq2 * SEq2 - dSEq3 * SEq3 - az

		// Functions from b-field
		float f4 = dbx * (0.5 - sSEq3 - SEq4 * SEq4) + dbz * (SEq2SEq4 - SEq1SEq3) - mx
		float f5 = dbx * (SEq2 * SEq3 - SEq1 * SEq4) + dbz * (SEq1 * SEq2 + SEq3 * SEq4) - my
		float f6 = dbx * (SEq1SEq3 + SEq2SEq4) + dbz * (0.5 - SEq2 * SEq2 - sSEq3) - mz

		// Jacobian entries
		float J1124 = dSEq3
		float J1223 = dSEq4
		float J1322 = dSEq1
		float J1421 = dSEq2
		float J32 = 2 * J1421
		float J33 = 2 * J1124
		float J41 = dbzSEq3
		float J42 = dbzSEq4
		float J43 = 2 * dbxSEq3 + dbzSEq1
		float J44 = 2 * dbxSEq4 - dbzSEq2
		float J51 = dbxSEq4 - dbzSEq2
		float J52 = dbxSEq3 + dbzSEq1
		float J53 = dbxSEq2 + dbzSEq4
		float J54 = dbxSEq1 - dbzSEq3
		float J61 = dbxSEq3
		float J62 = dbxSEq4 - 2 * dbzSEq2
		float J63 = dbxSEq1 - 2 * dbzSEq3
		float J64 = dbxSEq2

		// Gradient Descent Optimization
		// Gradients
		float SEqhatdot1 = -J1124 * f1 + J1421 * f2 - J41 * f4 - J51 * f5 + J61 * f6
		float SEqhatdot2 = J1223 * f1 + J1322 * f2 - J32 * f3 + J42 * f4 + J52 * f5 + J62 * f6
		float SEqhatdot3 = -J1322 * f1 + J1223 * f2 - J33 * f3 - J43 * f4 + J53 * f5 + J63 * f6
		float SEqhatdot4 = J1421 * f1 + J1124 * f2 - J44 * f4 - J54 * f5 + J64 * f6

		// Normalizing Gradients
		tempNorm = sqrt(SEqhatdot1 * SEqhatdot1 + SEqhatdot2 * SEqhatdot2 + SEqhatdot3 * SEqhatdot3 + SEqhatdot4 * SEqhatdot4)
		SEqhatdot1 /= tempNorm
		SEqhatdot2 /= tempNorm
		SEqhatdot3 /= tempNorm
		SEqhatdot4 /= tempNorm

		// Angular estimated direction of gyro error
		float wex = dSEq1 * SEqhatdot2 - dSEq2 * SEqhatdot1 - dSEq3 * SEqhatdot4 + dSEq4 * SEqhatdot3
		float wey = dSEq1 * SEqhatdot3 + dSEq2 * SEqhatdot4 - dSEq3 * SEqhatdot1 - dSEq4 * SEqhatdot2
		float wez = dSEq1 * SEqhatdot4 - dSEq2 * SEqhatdot3 + dSEq3 * SEqhatdot2 - dSEq4 * SEqhatdot1

		// Remove gyro bias
		wbx += wex * dt * zeta
		wby += wey * dt * zeta
		wbz += wez * dt * zeta
		wx -= wbx
		wy -= wby
		wz -= wbz

		// Quaternion rate of change (gyro)
		SEqdot1 = -hSEq2 * wx - hSEq3 * wy - hSEq4 * wz
		SEqdot2 = hSEq1 * wx + hSEq3 * wz - hSEq4 * wy
		SEqdot3 = hSEq1 * wy - hSEq2 * wz + hSEq4 * wx
		SEqdot4 = hSEq1 * wz + hSEq2 * wy - hSEq3 * wx

		// Update orientation quaternion
		SEq1 += (SEqdot1 - (beta * SEqhatdot1)) * dt
		SEq2 += (SEqdot2 - (beta * SEqhatdot2)) * dt
		SEq3 += (SEqdot3 - (beta * SEqhatdot3)) * dt
		SEq4 += (SEqdot4 - (beta * SEqhatdot4)) * dt

		// Normalize orientation quaternion
		// can SIMD this
		tempNorm = sqrt(SEq1 * SEq1 + SEq2 * SEq2 + SEq3 * SEq3 + SEq4 * SEq4)
		SEq1 /= tempNorm
		SEq2 /= tempNorm
		SEq3 /= tempNorm
		SEq4 /= tempNorm

		// b-field in earth frame
		float SEq1SEq2 = SEq1 * SEq2
		float SEq1SEq3 = SEq1 * SEq3
		float SEq1SEq4 = SEq1 * SEq4
		float SEq3SEq4 = SEq3 * SEq4
		float SEq2SEq3 = SEq2 * SEq3
		float SEq2SEq4 = SEq2 * SEq4

		float hx = dmx * (0.5 - SEq3 * SEq3 - SEq4 * SEq4) + dmy * (SEq2SEq3 - SEq1SEq4) + dmz * (SEq2SEq4 + SEq1SEq3)
		float hy = dmx * (SEq2SEq3 + SEq1SEq4) + dmy * (0.5 - SEq2 * SEq2 - SEq4 * SEq4) + dmz * (SEq3SEq4 - SEq1SEq2)
		float hz = dmx * (SEq2SEq4 - SEq1SEq3) + dmy * (SEq3SEq4 + SEq1SEq2) + dmz * (0.5 - SEq2 * SEq2 - SEq3 * SEq3)

		// Normalize flux vector to eliminate y component
		bx = sqrt(hx * hx + hy * hy)
		bz = hz
	}