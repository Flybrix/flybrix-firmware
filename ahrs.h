#ifndef SE_AHRS_H_
#define SE_AHRS_H_

void se_madgwick_ahrs_update_imu_with_mag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float delta_time, float beta, float q[4]);

void se_madgwick_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float delta_time, float beta, float q[4]);

void se_mahony_ahrs_update_imu_with_mag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float delta_time, float ki_2, float kp_2, float fb_i[3], float q[4]);

void se_mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float delta_time, float ki_2, float kp_2, float fb_i[3], float q[4]);

#endif /* end of include guard: SE_AHRS_H_ */
