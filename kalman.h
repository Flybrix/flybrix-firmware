#ifndef SE_KALMAN_H_
#define SE_KALMAN_H_

void se_kalman_predict(float deltaTime, float* state, float* covar);

void se_kalman_correct(float* state, float* covar, int coordinate, float value, float variance);

#endif /* end of include guard: SE_KALMAN_H_ */
