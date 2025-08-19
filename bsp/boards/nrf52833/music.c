//#include <stdint.h>
//#include <math.h>

//#define N 3                // 阵元数
//#define L 10               // 快拍数（降低计算量）
//#define ANGLE_RES 5        // 角度搜索步长（度）
//#define PI 3.1415926535f

//typedef struct {
//    float real;
//    float imag;
//} ComplexFloat;



//ComplexFloat X[N][L];      // 接收信号 (N x L)
//ComplexFloat Rxx[N][N];    // 协方差矩阵 (N x N)
//float eigenvalues[N];      // 特征值（实数）
//ComplexFloat U[N][N];      // 特征向量矩阵
//float music_spectrum[180 / ANGLE_RES]; // MUSIC空间谱


//ComplexFloat complex_add(ComplexFloat a, ComplexFloat b) {
//    ComplexFloat result;
//    result.real = a.real + b.real;
//    result.imag = a.imag + b.imag;
//    return result;
//}

//// 复数乘法
//ComplexFloat complex_mul(ComplexFloat a, ComplexFloat b) {
//    ComplexFloat result;
//    result.real = a.real * b.real - a.imag * b.imag;
//    result.imag = a.real * b.imag + a.imag * b.real;
//    return result;
//}

//// 复数共轭
//ComplexFloat complex_conj(ComplexFloat a) {
//    ComplexFloat result;
//    result.real = a.real;
//    result.imag = -a.imag;
//    return result;
//}

//// 复数标量乘法
//ComplexFloat complex_scale(ComplexFloat a, float scale) {
//    ComplexFloat result;
//    result.real = a.real * scale;
//    result.imag = a.imag * scale;
//    return result;
//}

//void compute_covariance(ComplexFloat X[N][L], ComplexFloat Rxx[N][N]) {
//    for (int i = 0; i < N; i++) {
//        for (int j = 0; j < N; j++) {
//            ComplexFloat sum = {0.0f, 0.0f};
//            for (int t = 0; t < L; t++) {
//                // sum += X[i][t] * conj(X[j][t])
//                sum = complex_add(sum, complex_mul(X[i][t], complex_conj(X[j][t])));
//            }
//            Rxx[i][j] = complex_scale(sum, 1.0f / L); // 平均
//        }
//    }
//}

//void jacobi_evd(ComplexFloat Rxx[N][N], float eigenvalues[N], ComplexFloat U[N][N]) {
//    // 初始化U为单位矩阵
//    for (int i = 0; i < N; i++) {
//        for (int j = 0; j < N; j++) {
//            U[i][j].real = (i == j) ? 1.0f : 0.0f;
//            U[i][j].imag = 0.0f;
//        }
//    }

//    int max_iter = 10;      // 减少迭代次数
//    float threshold = 1e-4f;

//    for (int iter = 0; iter < max_iter; iter++) {
//        // 找到最大非对角元素（仅比较实部）
//        float max_off = 0.0f;
//        int p = 0, q = 1;
//        for (int i = 0; i < N; i++) {
//            for (int j = i + 1; j < N; j++) {
//                float mag = sqrtf(Rxx[i][j].real * Rxx[i][j].real + Rxx[i][j].imag * Rxx[i][j].imag);
//                if (mag > max_off) {
//                    max_off = mag;
//                    p = i;
//                    q = j;
//                }
//            }
//        }
//        if (max_off < threshold) break;

//        // 计算旋转角度（简化版，仅处理实部）
//        float theta = 0.5f * atan2f(2.0f * Rxx[p][q].real, Rxx[p][p].real - Rxx[q][q].real);
//        float c = cosf(theta);
//        float s = sinf(theta);

//        // 更新Rxx
//        for (int k = 0; k < N; k++) {
//            ComplexFloat tmp_pk = Rxx[p][k];
//            ComplexFloat tmp_qk = Rxx[q][k];
//            Rxx[p][k] = complex_add(complex_scale(tmp_pk, c), complex_scale(tmp_qk, s));
//            Rxx[q][k] = complex_add(complex_scale(tmp_pk, -s), complex_scale(tmp_qk, c));
//        }

//        // 更新U
//        for (int k = 0; k < N; k++) {
//            ComplexFloat tmp_kp = U[k][p];
//            ComplexFloat tmp_kq = U[k][q];
//            U[k][p] = complex_add(complex_scale(tmp_kp, c), complex_scale(tmp_kq, s));
//            U[k][q] = complex_add(complex_scale(tmp_kp, -s), complex_scale(tmp_kq, c));
//        }
//    }

//    // 提取特征值（对角线实部）
//    for (int i = 0; i < N; i++) {
//        eigenvalues[i] = Rxx[i][i].real;
//    }
//}


//void music_algorithm(ComplexFloat U[N][N], int signal_num) {
//    int noise_space_dim = N - signal_num;
//    float min_angle = -90.0f;

//    for (int angle_idx = 0; angle_idx < (180 / ANGLE_RES); angle_idx++) {
//        float theta = min_angle + angle_idx * ANGLE_RES;
//        float phase = PI * sinf(theta * PI / 180.0f);  // 阵元间距d = λ/2

//        // 生成导向向量a(theta)
//        ComplexFloat a[N];
//        for (int i = 0; i < N; i++) {
//            float phi = i * phase;
//            a[i].real = cosf(phi);
//            a[i].imag = sinf(phi);
//        }

//        // 计算噪声子空间投影：P(theta) = 1 / (a^H * Un * Un^H * a)
//        float denominator = 0.0f;
//        for (int k = signal_num; k < N; k++) {
//            ComplexFloat dot_product = {0.0f, 0.0f};
//            for (int i = 0; i < N; i++) {
//                // dot_product += conj(U[i][k]) * a[i]
//                ComplexFloat conj_U = complex_conj(U[i][k]);
//                dot_product = complex_add(dot_product, complex_mul(conj_U, a[i]));
//            }
//            denominator += dot_product.real * dot_product.real + dot_product.imag * dot_product.imag;
//        }

//        music_spectrum[angle_idx] = (denominator < 1e-6f) ? 0.0f : 1.0f / denominator;
//    }
//}