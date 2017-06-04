void LSM9DS0::madgwickFilterUpdate() {
	// pre declaring variables for speed
	float hSEq0;
	float hSEq1;
	float hSEq2;
	float hSEq3;

	float dSEq0;
	float dSEq1; 
	float dSEq2; 
	float dSEq3;

	float SEqdot0 = 0.5f;
	float SEqdot1 = 0.5f;
	float SEqdot2 = 0.5f;
	float SEqdot3 = 0.5f;

	//code for low pass filter
	float prev_SEq[4] = {1, 0, 0, 0};
	float prev_prev_SEq[4] = {1, 0, 0, 0};
	float prev_prev_prev_SEq[4] = {1, 0, 0, 0};
	float prev_prev_prev_prev_SEq[4] = {1, 0, 0, 0};

	float weight_prev = 0.2;
	float weight_prev_prev = 0.2;
	float weight_prev_prev_prev = 0.2;
	float weight_prev_prev_prev_prev = 0.2;
	float conj = 1 - weight_prev - weight_prev_prev - weight_prev_prev_prev - weight_prev_prev_prev_prev;

	while(true) {

		// Keeping track of integration time step
		currTime = std::chrono::steady_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count() / 1000000.0f;
		prevTime = currTime;

		// Poll new values
		updateAccel();
		updateMag();
		updateGyro();

		/*********************************/
		/* Useful Variable Manipulations */
		/*********************************/

		hSEq0 = 0.5f * SEq[0];
		hSEq1 = 0.5f * SEq[1];
		hSEq2 = 0.5f * SEq[2];
		hSEq3 = 0.5f * SEq[3];
		
		dSEq0 = 2.0f * SEq[0];
		dSEq1 = 2.0f * SEq[1];
		dSEq2 = 2.0f * SEq[2];
		dSEq3 = 2.0f * SEq[3];

		float dbx = 2.0f * bx;
		float dbz = 2.0f * bz;

		float SEq0SEq2 = SEq[0] * SEq[2];
		float SEq1SEq3 = SEq[1] * SEq[3];

		/**************************/
		/* Beginning of Algorithm */
		/**************************/

		// Normalize acceleration and magnetometer values
		float sqrtOf = ax * ax + ay * ay + az * az;
		float tempNorm = invSqrt(sqrtOf);
		ax *= tempNorm;
		ay *= tempNorm;
		az *= tempNorm;

		sqrtOf = mx * mx + my * my + mz * mz;
		tempNorm = invSqrt(sqrtOf);
		mx *= tempNorm;
		my *= tempNorm;
		mz *= tempNorm;

		float dmx = 2 * mx;
		float dmy = 2 * my;
		float dmz = 2 * mz;


		// Multiple iterations of gradient descent //

		int GD_ITERATIONS = 10000; // iterations of gradient descent
		float SEqhat_k[4] = {SEq[0], SEq[1], SEq[2], SEq[3]}; // best guess at the optimized orientation is the last estimated one

		// Declaring auxiliary variables
		float dSEq0_k;
		float dSEq1_k;
		float dSEq2_k;
		float dSEq3_k;

		float sSEq2_k;

		float SEq1SEq3_k;
		float SEq0SEq2_k;

		float dbxSEq0_k;
		float dbxSEq1_k;
		float dbxSEq2_k;
		float dbxSEq3_k;

		float dbzSEq0_k;
		float dbzSEq1_k;
		float dbzSEq2_k;
		float dbzSEq3_k;

		float gradf0;
		float gradf1;
		float gradf2;
		float gradf3;

		float ALPHA = 20.0f; // Attenuation for mu
		float normSEqdot = SEqdot0 * SEqdot0 + SEqdot1 * SEqdot1 + SEqdot2 * SEqdot2 + SEqdot3 * SEqdot3;
		tempNorm = sqrt(normSEqdot);

		float mu = (1.0f / GD_ITERATIONS) * ALPHA * tempNorm * dt; // Initial value for mu

		for(int iteration = 0; iteration < GD_ITERATIONS; iteration++) {

			// Auxiliary variables for multiple iterations of gradient descent
			dSEq0_k = 2.0f * SEqhat_k[0];
			dSEq1_k = 2.0f * SEqhat_k[1];
			dSEq2_k = 2.0f * SEqhat_k[2];
			dSEq3_k = 2.0f * SEqhat_k[3];

			sSEq2_k = SEqhat_k[2] * SEqhat_k[2];

			SEq1SEq3_k = SEqhat_k[1] * SEqhat_k[3];
			SEq0SEq2_k = SEqhat_k[0] * SEqhat_k[2];

			dbxSEq0_k = dbx * SEqhat_k[0];
			dbxSEq1_k = dbx * SEqhat_k[1];
			dbxSEq2_k = dbx * SEqhat_k[2];
			dbxSEq3_k = dbx * SEqhat_k[3];

			dbzSEq0_k = dbz * SEqhat_k[0];
			dbzSEq1_k = dbz * SEqhat_k[1];
			dbzSEq2_k = dbz * SEqhat_k[2];
			dbzSEq3_k = dbz * SEqhat_k[3];

			// Combined cost function + Jacobian
			// Functions from g-field
			float f1_k = dSEq1_k * SEqhat_k[3] - dSEq0_k * SEqhat_k[2] - ax;
			float f2_k = dSEq0_k * SEqhat_k[1] + dSEq2_k * SEqhat_k[3] - ay;
			float f3_k = 1.0f - dSEq1_k * SEqhat_k[1] - dSEq2_k * SEqhat_k[2] - az;

			// Functions from b-field
			float f4_k = dbx * (0.5f - sSEq2_k - SEqhat_k[3] * SEqhat_k[3]) + dbz * (SEq1SEq3_k - SEq0SEq2_k) - mx;
			float f5_k = dbx * (SEqhat_k[1] * SEqhat_k[2] - SEqhat_k[0] * SEqhat_k[3]) + dbz * (SEqhat_k[0] * SEqhat_k[1] + SEqhat_k[2] * SEqhat_k[3]) - my;
			float f6_k = dbx * (SEq0SEq2_k + SEq1SEq3_k) + dbz * (0.5f - SEqhat_k[1] * SEqhat_k[1] - sSEq2_k) - mz;

			// Jacobian entries
			float J1124 = dSEq2_k;
			float J1223 = dSEq3_k;
			float J1322 = dSEq0_k;
			float J1421 = dSEq1_k;
			float J32 = 2.0f * J1421;
			float J33 = 2.0f * J1124;
			float J41 = dbzSEq2_k;
			float J42 = dbzSEq3_k;
			float J43 = 2.0f * dbxSEq2_k + dbzSEq0_k;
			float J44 = 2.0f * dbxSEq3_k - dbzSEq1_k;
			float J51 = dbxSEq3_k - dbzSEq1_k;
			float J52 = dbxSEq2_k + dbzSEq0_k;
			float J53 = dbxSEq1_k + dbzSEq3_k;
			float J54 = dbxSEq0_k - dbzSEq2_k;
			float J61 = dbxSEq2_k;
			float J62 = dbxSEq3_k - 2.0f * dbzSEq1_k;
			float J63 = dbxSEq0_k - 2.0f * dbzSEq2_k;
			float J64 = dbxSEq1_k;

			// Gradient Descent Optimization
			// Gradients
			gradf0 = -J1124 * f1_k + J1421 * f2_k - J41 * f4_k - J51 * f5_k + J61 * f6_k;
			gradf1 = J1223 * f1_k + J1322 * f2_k - J32 * f3_k + J42 * f4_k + J52 * f5_k + J62 * f6_k;
			gradf2 = -J1322 * f1_k + J1223 * f2_k - J33 * f3_k - J43 * f4_k + J53 * f5_k + J63 * f6_k;
			gradf3 = J1421 * f1_k + J1124 * f2_k - J44 * f4_k - J54 * f5_k + J64 * f6_k;

			// Normalizing Gradients
			sqrtOf = gradf0 * gradf0 + gradf1 * gradf1 + gradf2 * gradf2 + gradf3 * gradf3;
			tempNorm = invSqrt(sqrtOf);
			gradf0 *= tempNorm;
			gradf1 *= tempNorm;
			gradf2 *= tempNorm;
			gradf3 *= tempNorm;

			// Recalculating and renormalizing SEqhat_k
			SEqhat_k[0] = SEqhat_k[0] - mu * gradf0;
			SEqhat_k[1] = SEqhat_k[1] - mu * gradf0;
			SEqhat_k[2] = SEqhat_k[2] - mu * gradf0;
			SEqhat_k[3] = SEqhat_k[3] - mu * gradf0;

			sqrtOf = SEqhat_k[0] * SEqhat_k[0] + SEqhat_k[1] * SEqhat_k[1] + SEqhat_k[2] * SEqhat_k[2] + SEqhat_k[3] * SEqhat_k[3];
			tempNorm = invSqrt(sqrtOf);
			SEqhat_k[0] *= tempNorm;
			SEqhat_k[1] *= tempNorm;
			SEqhat_k[2] *= tempNorm;
			SEqhat_k[3] *= tempNorm;

		}

		float SEqhatdot0 = gradf0;
		float SEqhatdot1 = gradf1;
		float SEqhatdot2 = gradf2;
		float SEqhatdot3 = gradf3;

		// Angular estimated direction of gyro error
		float wex = dSEq0 * SEqhatdot1 - dSEq1 * SEqhatdot0 - dSEq2 * SEqhatdot3 + dSEq3 * SEqhatdot2;
		float wey = dSEq0 * SEqhatdot2 + dSEq1 * SEqhatdot3 - dSEq2 * SEqhatdot0 - dSEq3 * SEqhatdot1;
		float wez = dSEq0 * SEqhatdot3 - dSEq1 * SEqhatdot2 + dSEq2 * SEqhatdot1 - dSEq3 * SEqhatdot0;

		// Remove gyro bias
		gyroBiases[0] += wex * dt * ZETA;
		gyroBiases[1] += wey * dt * ZETA;
		gyroBiases[2] += wez * dt * ZETA;
		wx -= gyroBiases[0];
		wy -= gyroBiases[1];
		wz -= gyroBiases[2];

		// Quaternion rate of change (gyro)
		SEqdot0 = -hSEq1 * wx - hSEq2 * wy - hSEq3 * wz;
		SEqdot1 = hSEq0 * wx + hSEq2 * wz - hSEq3 * wy;
		SEqdot2 = hSEq0 * wy - hSEq1 * wz + hSEq3 * wx;
		SEqdot3 = hSEq0 * wz + hSEq1 * wy - hSEq2 * wx;

		// Update orientation quaternion
		SEq[0] += (SEqdot0 - (BETA * SEqhatdot0)) * dt;
		SEq[1] += (SEqdot1 - (BETA * SEqhatdot1)) * dt;
		SEq[2] += (SEqdot2 - (BETA * SEqhatdot2)) * dt;
		SEq[3] += (SEqdot3 - (BETA * SEqhatdot3)) * dt;

		// Normalize orientation quaternion
		sqrtOf = SEq[0] * SEq[0] + SEq[1] * SEq[1] + SEq[2] * SEq[2] + SEq[3] * SEq[3];
		tempNorm = invSqrt(sqrtOf);
		SEq[0] *= tempNorm;
		SEq[1] *= tempNorm;
		SEq[2] *= tempNorm;
		SEq[3] *= tempNorm;

		// b-field in earth frame
		float SEq0SEq1 = SEq[0] * SEq[1];
		SEq0SEq2 = SEq[0] * SEq[2];
		float SEq0SEq3 = SEq[0] * SEq[3];
		float SEq2SEq3 = SEq[2] * SEq[3];
		float SEq1SEq2 = SEq[1] * SEq[2];
		SEq1SEq3 = SEq[1] * SEq[3];

		float hx = dmx * (0.5f - SEq[2] * SEq[2] - SEq[3] * SEq[3]) + dmy * (SEq1SEq2 - SEq0SEq3) + dmz * (SEq1SEq3 + SEq0SEq2);
		float hy = dmx * (SEq1SEq2 + SEq0SEq3) + dmy * (0.5f - SEq[1] * SEq[1] - SEq[3] * SEq[3]) + dmz * (SEq2SEq3 - SEq0SEq1);
		float hz = dmx * (SEq1SEq3 - SEq0SEq2) + dmy * (SEq2SEq3 + SEq0SEq1) + dmz * (0.5f - SEq[1] * SEq[1] - SEq[2] * SEq[2]);

		// Normalize flux vector to eliminate y component
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;

		// For dynamic low pass filtering
		SEq[0] = SEq[0] * conj + weight_prev * prev_SEq[0] + weight_prev_prev * prev_prev_SEq[0] + weight_prev_prev_prev * prev_prev_prev_SEq[0] + weight_prev_prev_prev_prev * prev_prev_prev_prev_SEq[0];
		SEq[1] = SEq[1] * conj + weight_prev * prev_SEq[1] + weight_prev_prev * prev_prev_SEq[1] + weight_prev_prev_prev * prev_prev_prev_SEq[1] + weight_prev_prev_prev_prev * prev_prev_prev_prev_SEq[1];
		SEq[2] = SEq[2] * conj + weight_prev * prev_SEq[2] + weight_prev_prev * prev_prev_SEq[2] + weight_prev_prev_prev * prev_prev_prev_SEq[2] + weight_prev_prev_prev_prev * prev_prev_prev_prev_SEq[2];
		SEq[3] = SEq[3] * conj + weight_prev * prev_SEq[3] + weight_prev_prev * prev_prev_SEq[3] + weight_prev_prev_prev * prev_prev_prev_SEq[3] + weight_prev_prev_prev_prev * prev_prev_prev_prev_SEq[3];

		prev_prev_prev_prev_SEq[0] = prev_prev_prev_SEq[0];
		prev_prev_prev_prev_SEq[1] = prev_prev_prev_SEq[1];
		prev_prev_prev_prev_SEq[2] = prev_prev_prev_SEq[2];
		prev_prev_prev_prev_SEq[3] = prev_prev_prev_SEq[3];
		
		prev_prev_prev_SEq[0] = prev_prev_SEq[0];
		prev_prev_prev_SEq[1] = prev_prev_SEq[1];
		prev_prev_prev_SEq[2] = prev_prev_SEq[2];
		prev_prev_prev_SEq[3] = prev_prev_SEq[3];

		prev_prev_SEq[0] = prev_SEq[0];
		prev_prev_SEq[1] = prev_SEq[1];
		prev_prev_SEq[2] = prev_SEq[2];
		prev_prev_SEq[3] = prev_SEq[3];

		prev_SEq[0] = SEq[0];
		prev_SEq[1] = SEq[1];
		prev_SEq[2] = SEq[2];
		prev_SEq[3] = SEq[3];

		// Debug Prints
		calculateRPY();
		printf("dt: %f\n\n", dt);
	}
}