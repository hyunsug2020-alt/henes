#include "bisa/mpc_controller_cpp.hpp"
#include <unsupported/Eigen/MatrixFunctions>  // expm
#include <algorithm>
#include <cmath>

namespace bisa {

MPCControllerCpp::MPCControllerCpp() {
    current_kappa_ = 0.0;
}

MPCControllerCpp::~MPCControllerCpp() {}

void MPCControllerCpp::update_parameters(const LTVMPCParams& params) {
    params_ = params;
}

void MPCControllerCpp::reset_state() {
    current_kappa_ = 0.0;
}

// ===================================================================
// 올바른 Quaternion to Yaw 변환
// ===================================================================
double MPCControllerCpp::quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    // Hybrid yaw extraction:
    // - For standard quaternions (norm ~ 1): use quaternion->yaw conversion.
    // - For simulator-packed orientations (often x≈1.57, w≈1, z=yaw but NOT a valid quaternion): use yaw = z.
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);

    auto wrap = [](double a){
        while (a > M_PI)  a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    };

    // If it's not close to a unit quaternion, interpret as Euler-packed and take z as yaw.
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.15) {
        return wrap(q.z);
    }

    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrap(std::atan2(siny_cosp, cosy_cosp));
}

// ===================================================================
// Equation (2): 연속시간 시스템 행렬
// ===================================================================
// ẋ = Ac*x + Bc*u + Ec*z
// 
// x = [dr, theta, kappa, theta_r, kappa_r]^T
// u = kappa_dot (곡률 변화율)
// z = kappa_r_dot (기준 곡률 변화율)
// ===================================================================
void MPCControllerCpp::compute_system_matrices_continuous(
    double v,
    Eigen::MatrixXd& Ac,
    Eigen::VectorXd& Bc,
    Eigen::VectorXd& Ec) {
    
    // Ac(t) - 5x5 행렬
    Ac = Eigen::MatrixXd::Zero(5, 5);
    Ac(0, 1) = v;      // ḋr = v*(theta - theta_r)
    Ac(0, 3) = -v;
    Ac(1, 2) = v;      // θ̇ = v*kappa
    Ac(3, 4) = v;      // θ̇r = v*kappa_r
    
    // Bc - 5x1 벡터
    Bc = Eigen::VectorXd::Zero(5);
    Bc(2) = 1.0;       // κ̇ = u
    
    // Ec - 5x1 벡터
    Ec = Eigen::VectorXd::Zero(5);
    Ec(4) = 1.0;       // κ̇r = z
}

// ===================================================================
// Equation (4): 출력 행렬 (3원 차량 근사)
// ===================================================================
// y = [d1, d2, d3, kappa]^T
// di = dr + li*(theta - theta_r)
// 
// l1 = 0     (후륜 중심)
// l2 = l/2   (차량 중앙)
// l3 = l     (전륜 중심)
// ===================================================================
void MPCControllerCpp::compute_output_matrix(Eigen::MatrixXd& C) {
    C = Eigen::MatrixXd::Zero(4, 5);
    
    // d1 = dr
    C(0, 0) = 1.0;
    
    // d2 = dr + (l/2)*(theta - theta_r)
    C(1, 0) = 1.0;
    C(1, 1) = params_.l / 2.0;
    C(1, 3) = -params_.l / 2.0;
    
    // d3 = dr + l*(theta - theta_r)
    C(2, 0) = 1.0;
    C(2, 1) = params_.l;
    C(2, 3) = -params_.l;
    
    // kappa
    C(3, 2) = 1.0;
}

// ===================================================================
// Equation (5): 이산화 (Exact discretization)
// ===================================================================
// A(k) = e^(Ac*Ts)
// B(k) = Ac^(-1) * (A - I) * Bc
// E(k) = Ac^(-1) * (A - I) * Ec
// ===================================================================
void MPCControllerCpp::discretize_system(
    const Eigen::MatrixXd& Ac,
    const Eigen::VectorXd& Bc,
    const Eigen::VectorXd& Ec,
    double Ts,
    Eigen::MatrixXd& A,
    Eigen::MatrixXd& B,
    Eigen::MatrixXd& E) {

    const int n = Ac.rows();

    // Robust exact discretization using augmented matrix exponential (Van Loan method).
    // Avoids Ac.inverse() which is invalid when Ac is singular (common in this model).
    if (!std::isfinite(Ts) || Ts <= 1e-6) Ts = 0.05;

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + 2, n + 2);
    M.block(0, 0, n, n) = Ac;
    M.block(0, n, n, 1) = Bc;
    M.block(0, n + 1, n, 1) = Ec;

    Eigen::MatrixXd expM = (M * Ts).exp();
    A = expM.block(0, 0, n, n);
    B = expM.block(0, n, n, 1);
    E = expM.block(0, n + 1, n, 1);
}



// ===================================================================
// Equation (16-17): 배치 정식화 (Batch formulation)
// ===================================================================
// x_bar = A_bar*x0 + B_bar*u + E_bar*z
// y_bar = C_bar*x_bar
//
// A_bar: (5N x 5) - 상태 전파
// B_bar: (5N x N) - 입력 영향 (하삼각 구조)
// E_bar: (5N x N) - 외란 영향
// C_bar: (4N x 5N) - 출력 변환
// ===================================================================
void MPCControllerCpp::build_prediction_matrices(
    const std::vector<double>& v_profile,
    Eigen::MatrixXd& A_bar,
    Eigen::MatrixXd& B_bar,
    Eigen::MatrixXd& E_bar,
    Eigen::MatrixXd& C_bar) {
    
    int N = params_.N;
    double Ts = params_.Ts;
    
    // 각 시간 스텝에 대한 시스템 행렬 계산
    std::vector<Eigen::MatrixXd> A_list(N), B_list(N), E_list(N), C_list(N);
    
    for (int k = 0; k < N; ++k) {
        double v_k = (k < (int)v_profile.size()) ? v_profile[k] : v_profile.back();
        
        Eigen::MatrixXd Ac;
        Eigen::VectorXd Bc, Ec;
        compute_system_matrices_continuous(v_k, Ac, Bc, Ec);
        
        Eigen::MatrixXd A_k, B_k, E_k;
        discretize_system(Ac, Bc, Ec, Ts, A_k, B_k, E_k);
        
        Eigen::MatrixXd C_k;
        compute_output_matrix(C_k);
        
        A_list[k] = A_k;
        B_list[k] = B_k;
        E_list[k] = E_k;
        C_list[k] = C_k;
    }
    
    // A_bar 구성: x(k) = A(k-1) ... A(1) A(0) * x0
    A_bar = Eigen::MatrixXd::Zero(5 * N, 5);
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd prod = Eigen::MatrixXd::Identity(5, 5);
        for (int j = 0; j <= i; ++j) {
            prod = A_list[j] * prod;
        }
        A_bar.block(i * 5, 0, 5, 5) = prod;
    }
    
    // B_bar 구성: 하삼각 블록 행렬
    // x(k) = ... + ∑[j=0 to k-1] (∏[i=j+1 to k-1] A(i)) * B(j) * u(j)
    B_bar = Eigen::MatrixXd::Zero(5 * N, N);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j <= i; ++j) {
            Eigen::MatrixXd prod = Eigen::MatrixXd::Identity(5, 5);
            for (int k = j + 1; k <= i; ++k) {
                prod = A_list[k] * prod;
            }
            B_bar.block(i * 5, j, 5, 1) = prod * B_list[j];
        }
    }
    
    // E_bar 구성: B_bar와 동일한 구조
    E_bar = Eigen::MatrixXd::Zero(5 * N, N);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j <= i; ++j) {
            Eigen::MatrixXd prod = Eigen::MatrixXd::Identity(5, 5);
            for (int k = j + 1; k <= i; ++k) {
                prod = A_list[k] * prod;
            }
            E_bar.block(i * 5, j, 5, 1) = prod * E_list[j];
        }
    }
    
    // C_bar 구성: 블록 대각 행렬
    C_bar = Eigen::MatrixXd::Zero(4 * N, 5 * N);
    for (int i = 0; i < N; ++i) {
        C_bar.block(i * 4, i * 5, 4, 5) = C_list[i];
    }
}

// ===================================================================
// Equation (11): 비용 함수 행렬 (상태 공간에서 정의)
// ===================================================================
// J = x^T*Q*x + u^T*R*u
//
// Q: 상태 비용 (5N x 5N, 블록 대각)
// R: 입력 비용 (N x N, 대각)
//
// x = [dr, theta, kappa, theta_r, kappa_r]^T (5차원 상태)
// ===================================================================
void MPCControllerCpp::compute_cost_matrices(
    const std::vector<double>& v_profile,
    Eigen::MatrixXd& Q_bar,
    Eigen::MatrixXd& R_bar) {
    
    int N = params_.N;
    
    // Q_bar: 블록 대각 행렬 (5N x 5N) - 상태 공간
    Q_bar = Eigen::MatrixXd::Zero(5 * N, 5 * N);
    
    for (int k = 0; k < N; ++k) {
        double v_k = (k < (int)v_profile.size()) ? v_profile[k] : v_profile.back();
        
        // 속도 의존 가중치
        double wd_k = params_.wd;
        double wkappa_k = params_.wkappa + 0.5 * v_k;  // 고속에서 곡률 더 패널티
        
        // LTV core와 동일한 heading penalty: (theta - theta_r)^2
        Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(5, 5);
        Q_k(0, 0) = wd_k;
        Q_k(1, 1) = params_.wtheta;
        Q_k(1, 3) = -params_.wtheta;
        Q_k(2, 2) = wkappa_k;
        Q_k(3, 1) = -params_.wtheta;
        Q_k(3, 3) = params_.wtheta;
        
        Q_bar.block(k * 5, k * 5, 5, 5) = Q_k;
    }
    
    // R_bar: 입력 비용 (대각 행렬)
    // 최소값 보장으로 수치 안정성 향상
    double wu_safe = std::max(params_.wu, 1e-9);  // very small floor only for numerical stability
    R_bar = Eigen::MatrixXd::Identity(N, N) * wu_safe;
}

// ===================================================================
// 3점 곡률 계산
// ===================================================================
double MPCControllerCpp::compute_curvature_3points(
    double x1, double y1,
    double x2, double y2,
    double x3, double y3) {
    
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;
    
    double cross = dx1 * dy2 - dy1 * dx2;
    double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (d1 < 1e-6 || d2 < 1e-6) return 0.0;
    
    // Signed curvature with consistent 1/m unit.
    // κ = 2 * cross / (d1 * d2 * (d1 + d2))
    return (2.0 * cross) / (d1 * d2 * (d1 + d2) + 1e-6);
}

// [다음 파일에 계속...]
// ===================================================================
// Part 2: 보조 함수 및 상태 계산
// ===================================================================

// ===================================================================
// 가장 가까운 경로점 찾기
// ===================================================================
int MPCControllerCpp::find_closest_waypoint(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& path) {
    
    if (path.empty()) return -1;
    
    double x = current_pose.position.x;
    double y = current_pose.position.y;
    
    int closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.size(); ++i) {
        double dx = path[i].pose.position.x - x;
        double dy = path[i].pose.position.y - y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

// ===================================================================
// Equation (7): 상태 벡터 계산
// ===================================================================
// x = [dr, theta, kappa, theta_r, kappa_r]^T
//
// dr: 기준 경로로부터의 수직 거리 (lateral deviation)
// theta: 차량 방향각
// theta_r: 기준 경로 방향각
// kappa: 차량 곡률
// kappa_r: 기준 경로 곡률
// ===================================================================
Eigen::VectorXd MPCControllerCpp::compute_state_vector(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& reference_path) {
    
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(5);
    
    if (reference_path.size() < 3) {
        return x0;
    }
    
    // 현재 차량 상태
    double x_veh = current_pose.position.x;
    double y_veh = current_pose.position.y;
    double theta = quaternion_to_yaw(current_pose.orientation);
    
    // 가장 가까운 기준 경로점 찾기
    int closest_idx = find_closest_waypoint(current_pose, reference_path);
    
    // 기준 경로 상태
    double x_ref = reference_path[closest_idx].pose.position.x;
    double y_ref = reference_path[closest_idx].pose.position.y;
    double theta_ref = 0.0;
    {
        const int n = static_cast<int>(reference_path.size());
        int idx0 = std::max(0, std::min(closest_idx, n - 1));

        // Prefer path quaternion yaw (already generated in global_path_pub_multi.py).
        theta_ref = quaternion_to_yaw(reference_path[idx0].pose.orientation);

        // Fallback to geometric tangent when quaternion is unavailable/degenerate.
        int idx1 = (idx0 + 1) % n;
        const double dxr = reference_path[idx1].pose.position.x - reference_path[idx0].pose.position.x;
        const double dyr = reference_path[idx1].pose.position.y - reference_path[idx0].pose.position.y;
        if (std::hypot(dxr, dyr) > 1e-6) {
            const double tangent_yaw = std::atan2(dyr, dxr);
            // Blend on wrapped delta to avoid ±pi boundary artifacts.
            double dyaw = tangent_yaw - theta_ref;
            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            theta_ref += 0.3 * dyaw;
        }

        while (theta_ref > M_PI) theta_ref -= 2.0 * M_PI;
        while (theta_ref < -M_PI) theta_ref += 2.0 * M_PI;
    }
    
    // dr: 수직 거리 계산
    // 경로의 접선 벡터에 수직인 방향으로의 거리
    double dx = x_veh - x_ref;
    double dy = y_veh - y_ref;
    
    // 기준 경로의 법선 방향으로 투영
    double dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
    
    // kappa: 현재 차량 곡률 (저장된 값 사용)
    double kappa = current_kappa_;
    
    // kappa_r: 기준 경로 곡률 (3점 계산)
    double kappa_r = 0.0;
    if (closest_idx + 2 < (int)reference_path.size()) {
        double x1 = reference_path[closest_idx].pose.position.x;
        double y1 = reference_path[closest_idx].pose.position.y;
        double x2 = reference_path[closest_idx + 1].pose.position.x;
        double y2 = reference_path[closest_idx + 1].pose.position.y;
        double x3 = reference_path[closest_idx + 2].pose.position.x;
        double y3 = reference_path[closest_idx + 2].pose.position.y;
        
        kappa_r = compute_curvature_3points(x1, y1, x2, y2, x3, y3);
    }
    
    // 상태 벡터 구성
    x0(0) = dr;
    x0(1) = theta;
    x0(2) = kappa;
    x0(3) = theta_ref;
    x0(4) = kappa_r;
    
    return x0;
}

// ===================================================================
// Equation (7): 외란 신호 계산
// ===================================================================
// z(k) = dκr/dt = (κr(k+1) - κr(k)) / Ts
// ===================================================================
Eigen::VectorXd MPCControllerCpp::compute_disturbance_signal(
    const std::vector<geometry_msgs::msg::PoseStamped>& reference_path) {
    
    int N = params_.N;
    double Ts = params_.Ts;
    
    Eigen::VectorXd z = Eigen::VectorXd::Zero(N);
    
    if (reference_path.size() < 3) {
        return z;
    }
    
    // 각 예측 스텝에 대한 기준 곡률 계산
    std::vector<double> kappa_r_profile;
    
    for (size_t i = 0; i + 2 < reference_path.size(); ++i) {
        double x1 = reference_path[i].pose.position.x;
        double y1 = reference_path[i].pose.position.y;
        double x2 = reference_path[i + 1].pose.position.x;
        double y2 = reference_path[i + 1].pose.position.y;
        double x3 = reference_path[i + 2].pose.position.x;
        double y3 = reference_path[i + 2].pose.position.y;
        
        double kappa_r = compute_curvature_3points(x1, y1, x2, y2, x3, y3);
        kappa_r_profile.push_back(kappa_r);
    }
    
    // 외란 신호 z(k) = (κr(k+1) - κr(k)) / Ts
    for (int k = 0; k < N - 1; ++k) {
        if (k + 1 < (int)kappa_r_profile.size()) {
            z(k) = (kappa_r_profile[k + 1] - kappa_r_profile[k]) / Ts;
        }
    }
    
    return z;
}

// ===================================================================
// 속도 프로파일 생성 (곡률 기반)
// ===================================================================
std::vector<double> MPCControllerCpp::generate_velocity_profile(
    const std::vector<geometry_msgs::msg::PoseStamped>& reference_path) {
    
    int N = params_.N;
    std::vector<double> v_profile(N, params_.max_velocity);
    
    if (reference_path.size() < 3) {
        return v_profile;
    }
    
    // 각 예측 스텝에 대한 속도 계산
    for (int k = 0; k < N; ++k) {
        size_t idx = std::min((size_t)k, reference_path.size() - 3);
        
        double x1 = reference_path[idx].pose.position.x;
        double y1 = reference_path[idx].pose.position.y;
        double x2 = reference_path[idx + 1].pose.position.x;
        double y2 = reference_path[idx + 1].pose.position.y;
        double x3 = reference_path[idx + 2].pose.position.x;
        double y3 = reference_path[idx + 2].pose.position.y;
        
        double kappa_r = compute_curvature_3points(x1, y1, x2, y2, x3, y3);
        
        // 곡률에 따른 속도 감소
        // v = v_max / (1 + α*|κ|)
        double alpha = 3.0;
        double v_k = params_.max_velocity / (1.0 + alpha * std::abs(kappa_r));
        v_k = std::max(params_.min_velocity, std::min(v_k, params_.max_velocity));
        
        v_profile[k] = v_k;
    }
    
    return v_profile;
}

// ===================================================================
// Kamm's circle 기반 곡률 제한
// ===================================================================
// |κ| ≤ μ*g/v²
// ===================================================================
void MPCControllerCpp::compute_curvature_limits(
    double v,
    double& kappa_min,
    double& kappa_max) {
    
    kappa_max = params_.kappa_max_delta;
    kappa_min = params_.kappa_min_delta;

    // No tire friction model in this simulator. If mu<=0, skip Kamm's-circle limiting.
    if (params_.mu <= 0.0) {
        return;
    }
    
    if (v > 0.1) {
        // Kamm's circle 마찰 제약
        double v2 = v * v;
        double friction_limit = params_.mu * params_.g / (v2 + 0.01);
        
        kappa_max = std::min(kappa_max, friction_limit);
        kappa_min = std::max(kappa_min, -friction_limit);
    }
}

// ===================================================================
// Dense to Sparse 변환 (OSQP 입력용)
// OSQP는 P 행렬이 상삼각(upper triangular)이어야 함
// ===================================================================
Eigen::SparseMatrix<double> MPCControllerCpp::dense_to_sparse(
    const Eigen::MatrixXd& dense) {
    
    // 상삼각 부분만 추출 (OSQP 요구사항)
    // TriangularView -> Dense -> Sparse 순서로 변환
    Eigen::MatrixXd upper_dense = dense.triangularView<Eigen::Upper>();
    Eigen::SparseMatrix<double> sparse = upper_dense.sparseView();
    sparse.makeCompressed();
    return sparse;
}

// ===================================================================
// Part 3: QP 문제 구성 및 해결
// ===================================================================

// ===================================================================
// Equation (18), (23): QP 문제 구성
// ===================================================================
// 원래 문제:
//   minimize:  x^T*Q*x + u^T*R*u
//   subject to: x = A_bar*x0 + B_bar*u + E_bar*z
//               u_min <= u <= u_max
//               d_min <= C_bar*x <= d_max
//
// QP 형태로 변환:
//   minimize:  0.5*u^T*H*u + f^T*u
//   subject to: l <= A_qp*u <= u
//
// H = B_bar^T * Q_state * B_bar + R
// f = B_bar^T * Q_state * (A_bar*x0 + E_bar*z)
// ===================================================================
bool MPCControllerCpp::formulate_qp(
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& z,
    const std::vector<double>& v_profile,
    const CollisionConstraints& constraints,
    Eigen::SparseMatrix<double>& P,
    Eigen::VectorXd& q,
    Eigen::SparseMatrix<double>& A_qp,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u) {
    
    int N = params_.N;
    // 예측 행렬 계산
    Eigen::MatrixXd A_bar, B_bar, E_bar, C_bar;
    build_prediction_matrices(v_profile, A_bar, B_bar, E_bar, C_bar);
    (void)C_bar;
    
    // 비용 행렬 계산
    Eigen::MatrixXd Q_bar, R_bar;
    compute_cost_matrices(v_profile, Q_bar, R_bar);
    
    // 상태 예측: x_pred = A_bar*x0 + E_bar*z
    Eigen::VectorXd x_pred = A_bar * x0 + E_bar * z;
    
    // Equation (18): QP 변환 (state-space cost)
    // H = B^T * Q * B + R
    Eigen::MatrixXd H_dense = B_bar.transpose() * Q_bar * B_bar + R_bar;
    
    // 대칭화 (수치 오차 방지)
    H_dense = 0.5 * (H_dense + H_dense.transpose());
    
    // Regularization 추가 (PSD 보장)
    // P가 positive semi-definite이어야 convex QP
    int n = H_dense.rows();
    double eps = 1e-6;  // 작은 regularization으로 convexity 보장 + 해 왜곡 최소화
    H_dense += eps * Eigen::MatrixXd::Identity(n, n);
    
    // f = B^T * Q * (A*x0 + E*z)
    Eigen::VectorXd f = B_bar.transpose() * Q_bar * x_pred;
    
    // OSQP는 0.5*x^T*P*x + q^T*x 형태를 기대
    // 우리: x^T*H*x + 2*f^T*x
    // 변환: P = 2*H, q = 2*f
    P = dense_to_sparse(2.0 * H_dense);
    q = 2.0 * f;
    
    // ===================================================================
    // 제약 조건 구성
    // ===================================================================
    
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<double> lower_bounds, upper_bounds;
    
    int constraint_count = 0;
    
    // (1) Equation (9): 입력 제약
    // u_min <= u(k) <= u_max
    for (int k = 0; k < N; ++k) {
        triplets.push_back(Eigen::Triplet<double>(constraint_count, k, 1.0));
        lower_bounds.push_back(params_.u_min);
        upper_bounds.push_back(params_.u_max);
        constraint_count++;
    }
    
    // (2) 곡률 제약 - 간접적으로 입력 제약으로 처리됨
    
    // (3) 출력 제약 - 현재는 제거 (수치 안정성 우선)
    // TODO: 충돌 회피 제약은 안정화 후 추가
    
    // 제약 행렬 구성 (입력 제약만)
    A_qp.resize(constraint_count, N);
    A_qp.setFromTriplets(triplets.begin(), triplets.end());
    A_qp.makeCompressed();
    
    l = Eigen::Map<Eigen::VectorXd>(lower_bounds.data(), lower_bounds.size());
    u = Eigen::Map<Eigen::VectorXd>(upper_bounds.data(), upper_bounds.size());
    
    return true;
}

// ===================================================================
// OSQP로 QP 문제 해결
// ===================================================================
bool MPCControllerCpp::solve_qp_osqp(
    const Eigen::SparseMatrix<double>& P,
    const Eigen::VectorXd& q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& l,
    const Eigen::VectorXd& u,
    Eigen::VectorXd& solution) {
    
    c_int n = P.rows();
    c_int m = A.rows();
    
    // Eigen Sparse -> OSQP CSC format
    std::vector<c_float> P_data(P.nonZeros());
    std::vector<c_int> P_indices(P.nonZeros());
    std::vector<c_int> P_indptr(n + 1);
    
    int idx = 0;
    for (int k = 0; k < P.outerSize(); ++k) {
        P_indptr[k] = idx;
        for (Eigen::SparseMatrix<double>::InnerIterator it(P, k); it; ++it) {
            P_data[idx] = it.value();
            P_indices[idx] = it.row();
            idx++;
        }
    }
    P_indptr[n] = idx;
    
    std::vector<c_float> q_data(n);
    for (int i = 0; i < n; ++i) {
        q_data[i] = q(i);
    }
    
    std::vector<c_float> A_data(A.nonZeros());
    std::vector<c_int> A_indices(A.nonZeros());
    std::vector<c_int> A_indptr(n + 1);
    
    idx = 0;
    for (int k = 0; k < A.outerSize(); ++k) {
        A_indptr[k] = idx;
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
            A_data[idx] = it.value();
            A_indices[idx] = it.row();
            idx++;
        }
    }
    A_indptr[n] = idx;
    
    std::vector<c_float> l_data(m), u_data(m);
    for (int i = 0; i < m; ++i) {
        l_data[i] = l(i);
        u_data[i] = u(i);
    }
    
    // OSQP 데이터 구조체
    OSQPData data;
    data.n = n;
    data.m = m;
    data.P = csc_matrix(n, n, P_data.size(), P_data.data(), P_indices.data(), P_indptr.data());
    data.q = q_data.data();
    data.A = csc_matrix(m, n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data());
    data.l = l_data.data();
    data.u = u_data.data();
    
    // OSQP 설정
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;          // 로그 끄기
    settings.polish = 1;           // Solution polishing
    settings.max_iter = 4000;      // 최대 반복
    settings.eps_abs = 1e-4;
    settings.eps_rel = 1e-4;
    
    // Solver 생성
    OSQPWorkspace* work = nullptr;
    c_int exitflag = osqp_setup(&work, &data, &settings);
    
    if (exitflag != 0) {
        return false;
    }
    
    // 문제 해결
    osqp_solve(work);
    
    // 해가 수렴했는지 확인
    bool success = (work->info->status_val == OSQP_SOLVED || 
                   work->info->status_val == OSQP_SOLVED_INACCURATE);
    
    if (success) {
        solution = Eigen::VectorXd(n);
        for (int i = 0; i < n; ++i) {
            solution(i) = work->solution->x[i];
        }
    }
    
    // 정리
    osqp_cleanup(work);
    
    return success;
}

// ===================================================================
// 메인 제어 함수
// ===================================================================
ControlOutput MPCControllerCpp::compute_control(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path,
    const CollisionConstraints& constraints) {
    
    ControlOutput output;
    output.velocity = 0.0;
    output.angular_velocity = 0.0;
    
    if (local_path.size() < 5) {
        return output;
    }
    
    // 1. 상태 벡터 계산 (Equation 7)
    Eigen::VectorXd x0 = compute_state_vector(current_pose, local_path);
    
    // 2. 외란 신호 계산 (Equation 7)
    Eigen::VectorXd z = compute_disturbance_signal(local_path);
    
    // 3. 속도 프로파일 생성
    std::vector<double> v_profile = generate_velocity_profile(local_path);
    
    // 4. QP 문제 구성 (Equation 18, 23)
    Eigen::SparseMatrix<double> P, A_qp;
    Eigen::VectorXd q, l, u;
    
    bool formulated = formulate_qp(x0, z, v_profile, constraints, P, q, A_qp, l, u);
    
    if (!formulated) {
        return output;
    }
    
    // 5. QP 해결 (OSQP)
    Eigen::VectorXd u_solution;
    bool solved = solve_qp_osqp(P, q, A_qp, l, u, u_solution);
    
    if (!solved || u_solution.size() == 0) {
        return output;
    }
    
    // 6. 첫 번째 제어 입력 적용
    double u0 = u_solution(0);  // 곡률 변화율
    
    // 곡률 업데이트
    current_kappa_ += u0 * params_.Ts;
    
    // 곡률 제약 적용
    double kappa_min, kappa_max;
    compute_curvature_limits(v_profile[0], kappa_min, kappa_max);
    current_kappa_ = std::max(kappa_min, std::min(current_kappa_, kappa_max));
    
    // 7. 속도 및 각속도 계산
    double velocity = v_profile[0];
    double omega = velocity * current_kappa_;
    
    output.velocity = velocity;
    output.angular_velocity = omega;
    
    // 8. 예측 궤적 생성 (visualization)
    double x = current_pose.position.x;
    double y = current_pose.position.y;
    double yaw = quaternion_to_yaw(current_pose.orientation);
    
    for (int k = 0; k < params_.N; ++k) {
        x += velocity * std::cos(yaw) * params_.Ts;
        y += velocity * std::sin(yaw) * params_.Ts;
        yaw += omega * params_.Ts;
        
        output.predicted_trajectory.push_back({x, y, 0.0});
    }
    
    return output;
}

}  // namespace bisa
