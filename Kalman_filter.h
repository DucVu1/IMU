#ifndef _Kalman_h
#define _Kalman_h

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;

    double angle;
    double bias;
    double rate;

    double P[2][2];
    double K[2];
    double y;
    double S;
} Kalman;

void Kalman_init(Kalman *kalman) {
    kalman->Q_angle = 0.001;
    kalman->Q_bias = 0.003;
    kalman->R_measure = 0.03;

    kalman->angle = 0;
    kalman->bias = 0;

    kalman->P[0][0] = 0;
    kalman->P[0][1] = 0;
    kalman->P[1][0] = 0;
    kalman->P[1][1] = 0;
}

double Kalman_getAngle(Kalman *kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - kalman->bias;
    kalman->angle += dt * rate;

    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    kalman->S = kalman->P[0][0] + kalman->R_measure;
    kalman->K[0] = kalman->P[0][0] / kalman->S;
    kalman->K[1] = kalman->P[1][0] / kalman->S;

    double y = newAngle - kalman->angle;

    kalman->angle += kalman->K[0] * y;
    kalman->bias += kalman->K[1] * y;

    kalman->P[0][0] -= kalman->K[0] * kalman->P[0][0];
    kalman->P[0][1] -= kalman->K[0] * kalman->P[0][1];
    kalman->P[1][0] -= kalman->K[1] * kalman->P[0][0];
    kalman->P[1][1] -= kalman->K[1] * kalman->P[0][1];

    return kalman->angle;
}

void Kalman_setAngle(Kalman *kalman, double newAngle) {
    kalman->angle = newAngle;
}

double Kalman_getRate(Kalman *kalman) {
    return kalman->rate;
}

void Kalman_setQangle(Kalman *kalman, double newQ_angle) {
    kalman->Q_angle = newQ_angle;
}

void Kalman_setQbias(Kalman *kalman, double newQ_bias) {
    kalman->Q_bias = newQ_bias;
}

void Kalman_setRmeasure(Kalman *kalman, double newR_measure) {
    kalman->R_measure = newR_measure;
}

double Kalman_getQangle(Kalman *kalman) {
    return kalman->Q_angle;
}

double Kalman_getQbias(Kalman *kalman) {
    return kalman->Q_bias;
}

double Kalman_getRmeasure(Kalman *kalman) {
    return kalman->R_measure;
}

#endif
