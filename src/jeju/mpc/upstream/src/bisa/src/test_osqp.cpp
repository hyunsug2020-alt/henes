#include <iostream>
#include <vector>
#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

int main() {
    std::cout << "=== OSQP Test ===" << std::endl;
    
    // 간단한 QP 문제:
    // minimize: 0.5*x^T*P*x + q^T*x
    // subject to: l <= A*x <= u
    //
    // Example: min (x1-1)^2 + (x2-0.5)^2
    //          s.t. x1 + x2 = 1
    
    c_int n = 2;  // 변수 개수
    c_int m = 1;  // 제약 개수
    
    // P = [[2, 0], [0, 2]] (sparse format)
    std::vector<c_float> P_data = {2.0, 2.0};
    std::vector<c_int> P_indices = {0, 1};
    std::vector<c_int> P_indptr = {0, 1, 2};
    
    // q = [-2, -1]
    std::vector<c_float> q = {-2.0, -1.0};
    
    // A = [[1, 1]] (sparse format)
    std::vector<c_float> A_data = {1.0, 1.0};
    std::vector<c_int> A_indices = {0, 0};
    std::vector<c_int> A_indptr = {0, 1, 2};
    
    // l = u = [1] (equality constraint)
    std::vector<c_float> l = {1.0};
    std::vector<c_float> u = {1.0};
    
    // OSQP 데이터 구조체 설정
    OSQPData data;
    data.n = n;
    data.m = m;
    data.P = csc_matrix(n, n, P_data.size(), P_data.data(), P_indices.data(), P_indptr.data());
    data.q = q.data();
    data.A = csc_matrix(m, n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data());
    data.l = l.data();
    data.u = u.data();
    
    // OSQP 설정
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 1;
    settings.polish = 1;
    
    // Solver 생성
    OSQPWorkspace* work = nullptr;
    c_int exitflag = osqp_setup(&work, &data, &settings);
    
    if (exitflag != 0) {
        std::cerr << "OSQP setup failed!" << std::endl;
        return -1;
    }
    
    // 문제 해결
    osqp_solve(work);
    
    // 결과 출력
    std::cout << "Status: " << work->info->status << std::endl;
    std::cout << "Solution: x1 = " << work->solution->x[0] 
              << ", x2 = " << work->solution->x[1] << std::endl;
    std::cout << "Expected: x1 = 0.667, x2 = 0.333" << std::endl;
    
    // 정리
    osqp_cleanup(work);
    
    std::cout << "OSQP test completed successfully!" << std::endl;
    return 0;
}
