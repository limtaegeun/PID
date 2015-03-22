#define Kp  1
#define Ki   1
#define Kd  1
#define dt   0.01
 
double current_val;    // 현재값
double err, prev_err;  // 오차. 이전 오차
double I_err, D_err;    // 오차적분. 오차미분
double Kp_term, Ki_term, Kd_term;   // p항, i항, d항
double control;
 
err = AIM - current_val;   // 오차 = 목표치-현재값
Kp_term = Kp * err;         // p항 = Kp*오차
 
I_err += err * dt;             // 오차적분 += 오차*dt
Ki_term = Ki * I_err;        // i항 = Ki*오차적분
 
D_err = (err-prev_err)/dt  // 오차미분 = (현재오차-이전오차)/dt
Kd_term = Kd * D_err;      // d항 = Kd*오차미분
 
prev_err = err;   // 현재오차를 이전오차로
 
control = Kp_term + Ki_term + Kd_term;  // 제어량 = p항+i항+d항
