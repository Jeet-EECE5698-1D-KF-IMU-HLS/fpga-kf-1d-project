// hls/kf1d_float/kf1d_float.cpp
extern "C" {
void kf1d_float(
    const float *u_gyro,   // [N] rad/s
    const float *z_accel,  // [N] rad
    float dt,
    float Q,
    float R,
    float x0,
    float P0,
    int   N,
    float *x_out,          // [N]
    float *P_out           // [N]
) {
#pragma HLS INTERFACE m_axi     port=u_gyro  offset=slave bundle=gmem
#pragma HLS INTERFACE m_axi     port=z_accel offset=slave bundle=gmem
#pragma HLS INTERFACE m_axi     port=x_out   offset=slave bundle=gmem
#pragma HLS INTERFACE m_axi     port=P_out   offset=slave bundle=gmem

#pragma HLS INTERFACE s_axilite port=u_gyro  bundle=control
#pragma HLS INTERFACE s_axilite port=z_accel bundle=control
#pragma HLS INTERFACE s_axilite port=x_out   bundle=control
#pragma HLS INTERFACE s_axilite port=P_out   bundle=control
#pragma HLS INTERFACE s_axilite port=dt      bundle=control
#pragma HLS INTERFACE s_axilite port=Q       bundle=control
#pragma HLS INTERFACE s_axilite port=R       bundle=control
#pragma HLS INTERFACE s_axilite port=x0      bundle=control
#pragma HLS INTERFACE s_axilite port=P0      bundle=control
#pragma HLS INTERFACE s_axilite port=N       bundle=control
#pragma HLS INTERFACE s_axilite port=return  bundle=control

    float x = x0;
    float P = P0;

KF_LOOP:
    for (int k = 0; k < N; ++k) {
#pragma HLS PIPELINE II=1
        float ug = u_gyro[k];
        float za = z_accel[k];

        // Predict
        float x_pred = x + dt * ug;
        float P_pred = P + Q;

        // Update
        float denom = P_pred + R;
        float K     = P_pred / denom;
        x = x_pred + K * (za - x_pred);
        P = (1.0f - K) * P_pred;

        x_out[k] = x;
        P_out[k] = P;
    }
}
}
