#include "kalman.h"
#include "lapack.h"

void se_kalman_predict(float deltaTime, float* state, float* covar) {
    float F[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    static const float Q[9] = {0.06f, 0.0f, 0.0f, 0.0f, 0.04f, 0.0f, 0.0f, 0.0f, 0.01f};
    static const int n = 3;
    static const float f_one = 1.0f, f_zero = 0.0f;
    float buffer[9];

    if (!(deltaTime > 0.0f))
        return;

    /* p += v * dt + a * dt * dt / 2; F[0] = 1, F[3] = dt, F[6] = dt * dt / 2 */
    F[3] = deltaTime;
    F[6] = deltaTime * deltaTime * 0.5f;
    state[0] += F[3] * state[1] + F[6] * state[2];
    /* v += a * dt; F[1] = 0, F[4] = 1, F[7] = dt */
    F[7] = deltaTime;
    state[1] += F[7] * state[2];

    /* buffer = P * F' */
    Fgemm_("n", "t", &n, &n, &n, &f_one, covar, &n, F, &n, &f_zero, buffer, &n);

    /* P = Q */
    Flacpy_(" ", &n, &n, Q, &n, covar, &n);

    /* P = F * buffer + dT * P = F * P * F' + dt * Q */
    Fgemm_("n", "n", &n, &n, &n, &f_one, F, &n, buffer, &n, &deltaTime, covar, &n);
}

void se_kalman_correct(float* state, float* covar, int coordinate, float value, float variance) {
    static const int n = 3, i_one = 1;
    static const float f_one = 1.0f, f_minus_one = -1.0f;
    float y = value - state[coordinate];
    float k_opt[3], h_row[3];
    float s_inverse = 1.0f / (covar[coordinate * 4] + variance);
    int i;
    for (i = 0; i < 3; ++i) {
        k_opt[i] = covar[i + coordinate * 3] * s_inverse;
        state[i] += k_opt[i] * y;
        h_row[i] = covar[coordinate + 3 * i];
    }
    /*
     * P -= K * H * P
     * H has 1 row and 3 columns with value col == coordinate ? 1 : 0
     * Thus, H * P is the "coordinate"-th row of P
     */
    Fgemm_("n", "n", &n, &n, &i_one, &f_minus_one, k_opt, &n, h_row, &i_one, &f_one, covar, &n);
}
