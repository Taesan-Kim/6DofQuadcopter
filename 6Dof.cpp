#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

//포인터
//*p는 p의 객체
//count << &v : 주소를 얻는 연산자
//Vector3* p = 는 Vector3의 주소를 저장하는 변수
//p가 주소라고 할 때, p(주소)->x(객체) 는 (*p).x



// 3차원 벡터 및 기본 연산 구조체
struct Vector3 { 
    //struct is bundle of data. 
    //class와의 유일한 차이점은 struct의 기본 접근 지정자는 public이다.
    //class는 private이다.

    //sizeof(Vector3) = 24bytes...double은 8byte
    //double은 64비트 부동소수점이다. 15~16자리 정밀도를 가진다.

    double x = 0.0; //c++11부터 가능
    double y = 0.0;
    double z = 0.0;

    //operator+는 연산자 오버로딩
    //a+b 는 컴파일러가 a.operator+(b)로 바꾼다.

    //const Vector3& v에서 const는 읽기 전용이란 의미, 즉 입력만 받고 수정하지 않겠다.
    //&는 참조의 의미. 즉 주소만 전달하겠다.
    //뒤의 const는 operator+()에서 a+b라고 할 , a가 변경되지 않는다는 의미.
    //따라서 x=100을 하면 컴파일 에러이다.

    // 벡터 덧셈
    Vector3 operator+(const Vector3& v) const { 
        return { 
            x + v.x,
            y + v.y,
            z + v.z 
        }; 
    }
    // 스칼라곱
    Vector3 operator*(double scalar) const {
        return {
            x * scalar,
            y * scalar,
            z * scalar 
        }; 
    }
    // 벡터 E셈
    Vector3 operator-(const Vector3& v) const {
        return {
            x - v.x,
            y - v.y,
            z - v.z 
        };
    }
    // 스칼라 나눗셈
    Vector3 operator/(double scalar) const {
        return {
            x / scalar,
            y / scalar,
            z / scalar
        };
    }
    // 자기 자신에 더하기
    Vector3& operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    // 자기 자신에 빼기
    Vector3& operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    // 자기 자신에 곱하기
    Vector3& operator*=(double s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    // 자기 자신에 나누기
    Vector3& operator/=(double s)
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }
    // 내적
    double dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    // 외적
    Vector3 cross(const Vector3& v) const {
        return {
            y * v.z - v.y * z,
            -x * v.z + v.x * z,
            x * v.y - v.x * y
        };
    }
    // 크기
    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
        //std:: 내가 만든 함수와 표준 라이브러리의 함수의 혼동을 막기 위함
        //네임스페이스
    }
    // 크기 제곱
    double normSquared() const
    {
        return x * x + y * y + z * z;
    }
    // 단위 벡터
    Vector3 normalized() const {
        double n = norm();
        if (n == 0.0)
            return {0.0, 0.0, 0.0};
        return {
            x / n,
            y / n,
            z / n
            };
    }
};

// 6자유도 상태 상태 벡터 정의
struct QuadState {
    Vector3 position;      // 지구고정좌표계 (X, Y, Z)
    Vector3 velocity;      // 지구고정좌표계 속도
    Vector3 attitude;      // 오일러 각 (Roll, Pitch, Yaw)
    Vector3 angularVel;    // 동체고정좌표계 각속도 (p, q, r)
};

struct QuadDerivative {
    Vector3 velocity;       // 위치의 변화율 = 속도
    Vector3 accel;          // 속도의 변화율 = 가속도
    Vector3 angularVel;     // 자세각의 변화율 = 각속도
    Vector3 angularAccel;   // 각속도의 변화율 = 각가속도
};

// 물리적 상수 및 기체 파라미터
//namespace는 변수, 함수, 클래스 등을 담아두는 이름 공간
//이름이 같은 변수의 충돌을 막기 위함
namespace Param {
    const double g = 9.81;        // 중력가속도
    const double mass = 1.0;      // 기체 질량 (kg)
    const Vector3 I = { 0.0081, 0.0081, 0.0142 }; // 관성모멘트 (Ixx, Iyy, Izz)
    const double l = 0.25;        // 회전축에서 모터까지의 거리 (m)
    const double b = 5.42e-5;     // 추력 계수
    const double d = 1.1e-6;      // 항력 계수
}

// 1. 동역학 플랜 클래스
class QuadcopterDynamics {
private:
    QuadState state;

// getDerivative
// INPUT: motorInput[4], Initial State(QuadState)
// OUTPUT: Derivative of State(12차원)...위치의 변화율, 속도의 변화율, 자세각의 변화율, 각속도의 변화율
// Method
//        위치의 변화율: 현재속도
//        속도의 변화율: 모터추력을 통한 동체좌표계 추력 구하기 + 자세각을 이용하여 동체좌표계를 지구좌표계로 변환.
//        자세각의 변화율: 자이로센서를 통해 동체 각변화율을 얻는다 + 동체각변화율을 관성좌표계로 변환한다(회전이용 Yaw -> Pitch -> Roll) + 관성좌표계로 변환된 동체각변화율 요소들을 총합하고, 
//                                                        오일러좌표계 각변화율 요소들을 가중치들과 분리하여 오일러좌표계 각변화율과 동체각변화율 간의 행렬등식을 만든다 + 역함수를 취해 오일러좌표계 각변화율을 동체각변화율에 대한 식으로 나타낸다.
//        각속도의 변화율: 뉴턴-오일러 각운동량 방정식을 통한 유도 (토크, 회전관성, 각속도를 알면 구할 수 있다.)
    QuadDerivative getDerivative(const QuadState& s, const double motorInputs[4]) const{
        QuadDerivative deriv;

        // 4개 모터 입력으로부터 총 추력(Thrust) 및 3축 모멘트(Torque) 계산
        double F1 = motorInputs[0];
        double F2 = motorInputs[1];
        double F3 = motorInputs[2];
        double F4 = motorInputs[3];

        double totalThrust = F1 + F2 + F3 + F4;

        // 업데이트된 엑스형(X) 배치 동역학 토크 계산
        Vector3 torque;
        // 전방/후방, 좌/우 매핑에 맞춰 부호 정렬 (오른손 법칙 기준)
        torque.x = Param::l * (F2 + F1 - F4 - F3); // Roll
        torque.y = Param::l * (F3 + F1 - F2 - F4); // Pitch
        torque.z = (Param::d / Param::b) * (F1 - F2 + F3 - F4); // Yaw

        // 물리 법칙에 의한 가속도 및 각가속도 계산 (상태 s에 의존)
        // 위치의 변화율 = 현재 속도
        deriv.velocity = s.velocity;

        // 가속도 계산
        // 동체좌표계 추력을 지구 좌표계 추력으로 변환하는 회전행렬 연산을 위한 변수설정
        double c_phi = cos(s.attitude.x);   double s_phi = sin(s.attitude.x);   // Roll
        double c_theta = cos(s.attitude.y); double s_theta = sin(s.attitude.y); // Pitch
        double c_psi = cos(s.attitude.z);   double s_psi = sin(s.attitude.z);   // Yaw

        // 지구 좌표계로 투영된 추력 가속도 (F_I = R * [0, 0, totalThrust]^T)
        deriv.accel.x = (totalThrust / Param::mass) * (c_phi * s_theta * c_psi + s_phi * s_psi);
        deriv.accel.y = (totalThrust / Param::mass) * (c_phi * s_theta * s_psi - s_phi * c_psi);
        deriv.accel.z = (totalThrust / Param::mass) * (c_phi * c_theta) - Param::g;

        // 동체 각속도는 자이로 센서에서 읽은 동체 각속도이다.
        // 동체 각속도는 오일러 각의 미분값 변화율과 다르다.
        // 각속도를 측정하는 기준 축이 실시간으로 변하기 때문이다.
        // 그러므로 행렬 관계식을 통해 동체 각속도를 가지고 오일러 각의 변화율을 구한다.
        // 행렬 관계식은 오일러각변화율에서 각각의 element들을 Rotation을 이용하여 
        // 맨 처음 고정된 Inertial좌표계 기준으로의 값으로 변환 후 총합을 Body Frame로 표현한다. (T(phi, theta)구하기)
        // 역변환을 통해 동체각속도를 오일러 각 변화율로 변환시켜주는 행렬을 만든다.

        //자세각 변화율 = 오일러 키네마틱스 변환 반영
        double tan_theta = tan(s.attitude.y);
        double cos_theta = c_theta; // cos(s.attitude.y);
        if (std::abs(cos_theta) < 1e-4) cos_theta = 1e-4; // 짐벌락 방지 임계치

        // 고도화: 동체 각속도(p,q,r)를 오일러 각 변화율(dot_phi, dot_theta, dot_psi)로 변환
        deriv.angularVel.x = s.angularVel.x + s_phi * tan_theta * s.angularVel.y + c_phi * tan_theta * s.angularVel.z; // dot_phi
        deriv.angularVel.y = c_phi * s.angularVel.y - s_phi * s.angularVel.z; // dot_theta
        deriv.angularVel.z = (s_phi / c_theta) * s.angularVel.y + (c_phi / c_theta) * s.angularVel.z; // dot_psi
        // 이때, theta가 +- pi/2일 때, Gimbal Lock이 발생함을 주의한다. (짐벌락 방지 코드 추가 완.)                                                                                               //추후 쿼터니언 기반 동역학으로 해결해야 할 것!

        
        // 각가속도 계산 (Euler's equations for rigid body)
        deriv.angularAccel.x = (torque.x + (Param::I.y - Param::I.z) * state.angularVel.y * state.angularVel.z) / Param::I.x;
        deriv.angularAccel.y = (torque.y + (Param::I.z - Param::I.x) * state.angularVel.z * state.angularVel.x) / Param::I.y;
        deriv.angularAccel.z = (torque.z + (Param::I.x - Param::I.y) * state.angularVel.x * state.angularVel.y) / Param::I.z;

        return deriv;
    }
public:
    //QuadcopterDynamics 함수 생성 후 멤버 변수 state를 initialState로 직접 초기화하라.
    QuadcopterDynamics(QuadState initialState) 
        : state(initialState) //함수 호출이 아니다. state는 함수가 아닌, 멤버 변수이다.
        //즉, state 객체를 initialState로 생성한다는 의미
    {}

    // 모터 속도(RPM 또는 무차원 제어입력)를 받아 상태를 갱신 (Euler integration)
    

    // RK4 기반 상태 업데이트 Method
    //updateRK4
    //INPUT: motorInputs, dt
    //OUTPUT: 룬지쿠타를 이용하여 구한 최종 state
    void updateRK4(const double motorInputs[4], double dt) {
        // 연산을 위한 임시 상태 구조체들
        QuadState s_tmp;

        // k1 계산 (현재 시점의 기울기)
        QuadDerivative k1 = getDerivative(state, motorInputs);

        // k2 계산 (dt/2 시점의 예견된 기울기 1)
        s_tmp.position = state.position + k1.velocity * (dt * 0.5);
        s_tmp.velocity = state.velocity + k1.accel * (dt * 0.5);
        s_tmp.attitude = state.attitude + k1.angularVel * (dt * 0.5);
        s_tmp.angularVel = state.angularVel + k1.angularAccel * (dt * 0.5);
        QuadDerivative k2 = getDerivative(s_tmp, motorInputs);

        // k3 계산 (dt/2 시점의 예견된 기울기 2)
        s_tmp.position = state.position + k2.velocity * (dt * 0.5);
        s_tmp.velocity = state.velocity + k2.accel * (dt * 0.5);
        s_tmp.attitude = state.attitude + k2.angularVel * (dt * 0.5);
        s_tmp.angularVel = state.angularVel + k2.angularAccel * (dt * 0.5);
        QuadDerivative k3 = getDerivative(s_tmp, motorInputs);

        // k4 계산 (dt 시점의 예견된 기울기)
        s_tmp.position = state.position + k3.velocity * dt;
        s_tmp.velocity = state.velocity + k3.accel * dt;
        s_tmp.attitude = state.attitude + k3.angularVel * dt;
        s_tmp.angularVel = state.angularVel + k3.angularAccel * dt;
        QuadDerivative k4 = getDerivative(s_tmp, motorInputs);

        // 최종 가중평균 적분 (RK4 공식)
        state.position = state.position + (k1.velocity + k2.velocity * 2.0 + k3.velocity * 2.0 + k4.velocity) * (dt / 6.0);
        state.velocity = state.velocity + (k1.accel + k2.accel * 2.0 + k3.accel * 2.0 + k4.accel) * (dt / 6.0);
        state.attitude = state.attitude + (k1.angularVel + k2.angularVel * 2.0 + k3.angularVel * 2.0 + k4.angularVel) * (dt / 6.0);
        state.angularVel = state.angularVel + (k1.angularAccel + k2.angularAccel * 2.0 + k3.angularAccel * 2.0 + k4.angularAccel) * (dt / 6.0);
    }

    QuadState getState() const { return state; }
};

// 2. 2차 Low pass filter
                                                    //Verilog 상에서 곱셈기 5개를 동시에 병렬로 가동시키고 그 결과들을 하나의 고속 가산기 트리로 묶으면, 1~2블록 만에 2차 필터 연산을 완전히 끝낼 수 있다.

class BiquadFilter {
private:
    // 필터 계수
    double b0, b1, b2;
    double a1, a2;

    // 과거 데이터 저장 변수 (Private 내부 캡슐화로 상태 유지)
    double x1, x2; // 입력의 과거 값 (x[t-1], x[t-2])
    double y1, y2; // 출력의 과거 값 (y[t-1], y[t-2])

public:
    BiquadFilter() : b0(1), b1(0), b2(0), a1(0), a2(0), x1(0), x2(0), y1(0), y2(0) {}

    // 필터 계수 설정 함수 (외부에서 계산된 계수를 주입.)
                                                                                            //추후 활용을 위해 차단 주파수만 입력하면 실시간으로 피터 계수를 직접 계산하는 프로그램도 추가 필요.
    void setCoefficients(double _b0, double _b1, double _b2, double _a1, double _a2){
        b0 = _b0; b1 = _b1; b2 = _b2;
        a1 = _a1; a2 = _a2;
    }

    void reset() {
        x1 = x2 = y1 = y2 = 0.0;
    }

    // 필터링 실행 함수
    double update(double x) {
        // 차분 방정식 연산: y[t] = b0*x[t] + b1*x[t-1] + b2*x[t-2] - a1*y[t-1] - a2*y[t-2]
        double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

        // 과거 상태 업데이트 (메모리 시프트)
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1= y;

        return y;
    }
};


// 3. 제어기 클래스 (추후 가벼운 룩업 테이블 기반 Explicit MPC 등으로 대체 가능)
class PIDCore {
private:
    // PID 게인 파라미터
    double kp;
    double ki;
    double kd;

    // 제어 상태 변수
    double prevError;
    double integral;

    // 제어 제약 조건 파라미터 (물리적 한계 모사)
    double integralLimit; // Anti-windup을 위한 적분 한계치
    double outputLimit;    // 최종 출력(토크/추력) 한계

    // D항 저역통과필터(LPF)를 위한 이전 D항 값 및 필터 계수
    double prevDTerm;
    double lpfAlpha;        // 0.0 ~ 1.0 (낮을수록 필터링 강함.)
    // lpfAlpha는 비율이다. 즉, 이전 미분값과 Raw미분값을 일정 비율로 섞은 값이 LPF필터값이 되는데,
    // 여기서 lpfAlpha는 Raw미분값이 차지하게 될 비율이다.

    // D항 2차 저역통과필터를 위한 인스턴스
    BiquadFilter dTermFilter; //기존에 생성한 BiquadFilter 클래스를 dTermFilter변수로 불러온다.

    // 첫 루프 예외 처리용 플래그
    bool isFirstStep;

public:
    PIDCore(double p, double i, double d, double iLimit double oLimit, double alpha = 1.0)
        : kp(p), ki(i), kd(d), 
        prevError(0.0), integral(0.0), prevDTerm(0.0),
        integralLimit(iLimit), outputLimit(oLimit), lpfAlpha(alpha),
        // 100Hz 루프에서 20Hz 이하만 통과시키는 2차 LPF 계수 세팅 가정
        dTermFilter.setCoefficients(0.0913, 0.1826, 0.0913, -1.1322, 0.4974),
        isFirstStep(true) {}

    // 제어기 초기화 (기체가 리셋되거나 착륙 후 재이륙 시 반드시 호출)
    void reset() {
        prevError = 0.0;
        integral = 0.0;
        prevDTerm = 0.0;
        isFirstStep = true;
    }

    // 제어 입력 계산 함수 (Discrete PID)
                                                                                // Verilog HDL을 설계할 때는 곱셈기, 누산기, 감산기를 동시에 한 클락 내에 연산을 끝내도록 파이프라인을 짜야 한다!
    double compute(double target, double current, double dt) {
        if (dt <= 0.0) return 0.0;

        // 1. 오차 계산 (Error)
        double error = target - current;

        // 2. P 항 (Proportional)
        double pTerm = kp * error;

        // 3. I 항 (Integral) & Anti-Windup 처리
        integral += error * dt;
        integral = std::max(-integralLimit, std::min(integralLimit, integral)); // 포화 방지
        double iTerm = ki * integral;

        // 4. D 항 (Derivative) & 1차 LPF
        //double dTerm = 0.0;
        //if (isFirstStep) {
            // 첫 번째 스텝에서 미분 폭발을 막기 위해 D항을 0으로 처리
            //isFirstStep = false;
        //} else {
            // 정석 미분 연산
            //double rawDTerm = kd * ((error - prevError) / dt);
            //객체지향: error, preError를 불러와 계산할 때 PIDcore class를 선언할 때 할당한 메모리에서 가져온다.
            //compute함수를 실행할 때, 함수 내부에서 preError가 업데이트 되고, 즉시 PIDcore class의 메모리에 저장된다.

            // LPF적용: D_filtered = alpha * D_raw + (1 - alpha) * D_previous
            //dTerm = lpfAlpha * rawDTerm + (1.0 - lpfAlpha) * prevDTerm;
        //}

        // 4.1 D 항 & 2차 LPF
        double dTerm = 0.0;
        if (!isFirstStep) {
            double rawDTerm = kd * ((error - prevError) / dt);

            // 2차 필터 통과
            dTerm = dTermFilter.update(rawDTerm);
        }
        

        // 다음 스텝을 위해 상태 업데이트 (Private 변수에 안전하게 저장)
        prevError = error;
        prevDTerm = dTerm;

        // 5. 최종 PID 제어 출력 & 출력 포화를 제한
        double totalOutput = pTerm + iTerm + dTerm;
        totalOutput = std::max(-outputLimit, std::min(outputLimit, totalOutput));
        
        return totalOutput;
    }
};


// 메인 컨트롤러 클래스
class QuadController {
private:
    // 위치(외곽) 제어기: X, Y, Z 고도
    PIDCore x_pos{1.5, 0.0, 1.0, 0.0};
    PIDCore y_pos{1.5, 0.0, 1.0, 0.0};
    PIDCore z_pos{25.0, 2.0, 10.0, 5.0}; // 고도는 중력을 이겨야 하므로 게인을 크게 설정

    // 자세(내곽) 제어기: Roll, Pitch, Yaw
    PIDCore roll_rate{6.0, 0.5, 2.0, 1.0};
    PIDCore pitch_rate{6.0, 0.5, 2.0, 1.0};
    PIDCore yaw_rate{4.0, 0.1, 1.0, 0.5};

public:
    // 메인 루프에서 호출되는 제어 입력 계산 함수
    void computeControlInputs(const QuadState& current, const QuadState& target, double outInputs[4], double dt) {
        // --- 1. 바깥쪽 루프 (Position Loop) ---
        // 위치 오차를 바탕으로 기체가 생성해야 할 목표 가속도/기울기(목표 오일러 각) 계산
        // 단순화를 위해 X 위치 오차는 Pitch에, Y 위치 오차는 Roll에 매핑 (방향 부호 주의)

        // compute: 제어 입력 계산 함수 (Discrete PID)
        double target_roll  = -y_pos.compute(target.position.y, current.position.y, dt);
        double target_pitch =  x_pos.compute(target.position.x, current.position.x, dt);
        double target_yaw   =  target.attitude.z; // 우선 목표 Yaw는 그대로 추종하도록 설정
        
        // 고도 제어 입력 (Base Thrust)
        double thrustCmd = z_pos.compute(target.position.z, current.position.z, dt);
        // 호버링을 위한 기본 중력 보상 평형 상태 추력 추가
        double hoverThrust = (Param::mass * Param::g);
        double totalThrust = hoverThrust + thrustCmd;

        // --- 2. 안쪽 루프 (Attitude Loop) ---
        // 목표 오일러 각과 현재 오일러 각의 오차를 바탕으로 원하는 3축 토크 명령 계산
        double rollTorque  = roll_rate.compute(target_roll,  current.attitude.x, dt);
        double pitchTorque = pitch_rate.compute(target_pitch, current.attitude.y, dt);
        double yawTorque   = yaw_rate.compute(target_yaw,   current.attitude.z, dt);

        // --- 3. 모터 믹서 (Motor Mixer) ---
        // 엑스형(X) 구조 토크 공식을 역산하여 4개 모터의 개별 추력 분배
        // 각 모터에 할당될 추력 F = (Thrust 분배) + (토크 분배 기여도)
        // 십자 성분 분산율(l)과 항력 계수 비율 반영
        double u_t = totalThrust / 4.0;
        double u_r = rollTorque / (4.0 * Param::l);
        double u_p = pitchTorque / (4.0 * Param::l);
        double u_y = yawTorque / (4.0 * (Param::d / Param::b));

        // 엑스형(X) 모터 믹싱 맵핑 (토크 방정식의 부호와 완벽히 대칭 매칭)
        outInputs[0] = u_t + u_r + u_p + u_y; // 후우 (1번 모터, CCW)
        outInputs[1] = u_t + u_r - u_p - u_y; // 전우 (2번 모터, CW)
        outInputs[2] = u_t - u_r + u_p + u_y; // 전좌 (3번 모터, CCW)
        outInputs[3] = u_t - u_r - u_p - u_y; // 후좌 (4번 모터, CW)

        // 하드웨어 보호를 위한 모터 출력 Max/Min Saturation 제한 (음수 방지)
        for(int i = 0; i < 4; ++i) {
            if(outInputs[i] < 0.0) outInputs[i] = 0.0;
        }
    }
};






// 3. 메인 시뮬레이션 루프 매니저
int main() {
    QuadState initState;
    initState.position = { 0.0, 0.0, 0.0 }; // 지면에서 시작

    QuadState targetState;
    targetState.position = { 0.0, 0.0, 10.0 }; // 목표 고도 10m

    QuadcopterDynamics quad(initState);
    QuadController controller;
    

    double dt = 0.01; // 100Hz 연산
    double time = 0.0;
    double motorInputs[4] = { 0, 0, 0, 0 };

    std::cout << "--- 시뮬레이션 시작 ---" << std::endl;
    for (int step = 0; step < 200; ++step) { // 2초간 시뮬레이션
        QuadState current = quad.getState();

        // 1. 제어 입력 계산
        controller.computeControlInputs(current, targetState, motorInputs, dt);

        // 2. 물리 엔진 업데이트
        quad.update(motorInputs, dt);

        time += dt;
        if (step % 20 == 0) { // 0.2초마다 터미널에 로그 출력
            std::cout << "Time: " << time << "s | Z-Position: " << current.position.z
                << "m | Z-Velocity: " << current.velocity.z << "m/s" << std::endl;
        }
    }
    return 0;
}


