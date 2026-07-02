#include <iostream>
#include <cmath>
#include <vector>

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
    //뒤의 const는 operator+()에서 a+b라고 할 떄, a가 변경되지 않는다는 의미.
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
    // 벡터 뺼셈
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

public:
    //QuadcopterDynamics 함수 생성 후 멤버 변수 state를 initialState로 직접 초기화하라.
    QuadcopterDynamics(QuadState initialState) 
        : state(initialState) //함수 호출이 아니다. state는 함수가 아닌, 멤버 변수이다.
        //즉, state 객체를 initialState로 생성한다는 의미
    {}

    // 모터 속도(RPM 또는 무차원 제어입력)를 받아 상태를 갱신 (Euler integration)
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

        // 자세의 변화율 = 현재 각속도
        // 현재 각속도는 자이로 센서에서 읽은 동체 각속도이다.
        // 동체 각속도는 오일러 각의 미분값 변화율과 다르다.
        // 각속도를 측정하는 기준 축이 실시간으로 변하기 때문이다.
        // 그러므로 행렬 관계식을 통해 동체 각속도를 가지고 오일러 각의 변화율을 구한다.
        deriv.angularVel = s.angularVel;

        // 각가속도 계산 (Euler's equations for rigid body)
        deriv.angularAccel.x = (torque.x + (Param::I.y - Param::I.z) * state.angularVel.y * state.angularVel.z) / Param::I.x;
        deriv.angularAccel.y = (torque.y + (Param::I.z - Param::I.x) * state.angularVel.z * state.angularVel.x) / Param::I.y;
        deriv.angularAccel.z = (torque.z + (Param::I.x - Param::I.y) * state.angularVel.x * state.angularVel.y) / Param::I.z;

        return deriv;
    }

    // RK4 기반 상태 업데이트 Method
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




// 2. 제어기 클래스 (추후 가벼운 룩업 테이블 기반 Explicit MPC 등으로 대체 가능)
class PIDController {
public:
    void computeControlInputs(const QuadState& current, const QuadState& target, double outInputs[4]) {
        // 고도(Z) 제어를 위한 아주 단순한 P 제어 예시
        double z_error = target.position.z - current.position.z;
        double baseThrust = Param::mass * Param::g / 4.0; // 호버링 추력
        double controlEffort = z_error * 2.0;            // P 게인 = 2.0

        for (int i = 0; i < 4; ++i) {
            outInputs[i] = baseThrust + controlEffort;
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
    PIDController controller;

    double dt = 0.01; // 100Hz 연산
    double time = 0.0;
    double motorInputs[4] = { 0, 0, 0, 0 };

    std::cout << "--- 시뮬레이션 시작 ---" << std::endl;
    for (int step = 0; step < 200; ++step) { // 2초간 시뮬레이션
        QuadState current = quad.getState();

        // 1. 제어 입력 계산
        controller.computeControlInputs(current, targetState, motorInputs);

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