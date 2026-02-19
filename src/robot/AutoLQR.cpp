#include <robot/AutoLQR.h>
#include <math.h>

AutoLQR::AutoLQR(int stateSize, int controlSize)
    : stateSize(stateSize)
    , controlSize(controlSize)
    , A(nullptr)
    , B(nullptr)
    , Q(nullptr)
    , R(nullptr)
    , K(nullptr)
    , state(nullptr)
    , P(nullptr)
    , solverMaxIterations(100)
    , solverTolerance(1e-6f)
{
    if (stateSize > 0 && controlSize > 0) {
        A = new double[stateSize * stateSize]();
        B = new double[stateSize * controlSize]();
        Q = new double[stateSize * stateSize]();
        R = new double[controlSize * controlSize]();
        K = new double[controlSize * stateSize]();
        state = new double[stateSize]();
        P = new double[stateSize * stateSize](); // Store Riccati solution
    }
}

AutoLQR::~AutoLQR()
{
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] K;
    delete[] state;
    delete[] P;
}

bool AutoLQR::setStateMatrix(const double* inputA)
{
    if (!inputA || !A)
        return false;
    memcpy(A, inputA, sizeof(double) * stateSize * stateSize);
    return true;
}

bool AutoLQR::setInputMatrix(const double* inputB)
{
    if (!inputB || !B)
        return false;
    memcpy(B, inputB, sizeof(double) * stateSize * controlSize);
    return true;
}

bool AutoLQR::setCostMatrices(const double* inputQ, const double* inputR)
{
    if (!inputQ || !inputR || !Q || !R)
        return false;
    memcpy(Q, inputQ, sizeof(double) * stateSize * stateSize);
    memcpy(R, inputR, sizeof(double) * controlSize * controlSize);
    return true;
}

void AutoLQR::setGains(const double* inputK)
{
    if (!inputK || !K)
        return;
    memcpy(K, inputK, sizeof(double) * controlSize * stateSize);
}

bool AutoLQR::computeGains()
{
    return computeGainMatrix();
}

bool AutoLQR::computeGains(int maxIterations, double tolerance)
{
    // Temporarily set solver parameters and compute
    int oldMaxIter = solverMaxIterations;
    double oldTol = solverTolerance;
    
    solverMaxIterations = maxIterations;
    solverTolerance = tolerance;
    
    bool result = computeGainMatrix();
    
    // Restore original parameters
    solverMaxIterations = oldMaxIter;
    solverTolerance = oldTol;
    
    return result;
}

void AutoLQR::setSolverParameters(int maxIterations, double tolerance)
{
    if (maxIterations > 0) {
        solverMaxIterations = maxIterations;
    }
    if (tolerance > 0) {
        solverTolerance = tolerance;
    }
}

int AutoLQR::getMaxIterations() const
{
    return solverMaxIterations;
}

double AutoLQR::getTolerance() const
{
    return solverTolerance;
}

void AutoLQR::updateState(const double* currentState)
{
    if (!currentState || !state)
        return;
    memcpy(state, currentState, sizeof(double) * stateSize);
}

void AutoLQR::calculateControl(double* controlOutput)
{
    if (!controlOutput || !K || !state)
        return;

    // Initialize control outputs to zero
    for (int i = 0; i < controlSize; i++) {
        controlOutput[i] = 0;
    }

    // u = -K·x
    for (int i = 0; i < controlSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            controlOutput[i] -= K[i * stateSize + j] * state[j];
        }
    }
}

void AutoLQR::matrixMultiply(const double* m1, const double* m2, double* result, int rows1, int cols1, int cols2)
{
    if (!m1 || !m2 || !result)
        return;

    // Initialize result matrix to zero
    for (int i = 0; i < rows1 * cols2; i++) {
        result[i] = 0;
    }

    for (int i = 0; i < rows1; i++) {
        for (int j = 0; j < cols2; j++) {
            for (int k = 0; k < cols1; k++) {
                result[i * cols2 + j] += m1[i * cols1 + k] * m2[k * cols2 + j];
            }
        }
    }
}

void AutoLQR::matrixAdd(const double* m1, const double* m2, double* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    for (int i = 0; i < rows * cols; i++) {
        result[i] = m1[i] + m2[i];
    }
}

void AutoLQR::matrixSubtract(const double* m1, const double* m2, double* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    for (int i = 0; i < rows * cols; i++) {
        result[i] = m1[i] - m2[i];
    }
}

void AutoLQR::transposeMatrix(const double* matrix, double* result, int rows, int cols)
{
    if (!matrix || !result)
        return;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[j * rows + i] = matrix[i * cols + j];
        }
    }
}

// Matrix inversion for small matrices (1x1, 2x2, 3x3) using closed-form
// and Gauss-Jordan elimination for larger matrices (4x4 and above)
bool AutoLQR::invertMatrix(const double* matrix, double* result, int n)
{
    if (!matrix || !result)
        return false;

    // Use closed-form solutions for small matrices (faster, no heap allocation)
    if (n == 1) {
        if (fabs(matrix[0]) < 1e-6)
            return false;
        result[0] = 1.0f / matrix[0];
        return true;
    } else if (n == 2) {
        double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
        if (fabs(det) < 1e-6)
            return false;
        double invDet = 1.0f / det;
        result[0] = matrix[3] * invDet;
        result[1] = -matrix[1] * invDet;
        result[2] = -matrix[2] * invDet;
        result[3] = matrix[0] * invDet;
        return true;
    } else if (n == 3) {
        // 3x3 Matrix inversion using Sarrus rule
        double a = matrix[0], b = matrix[1], c = matrix[2];
        double d = matrix[3], e = matrix[4], f = matrix[5];
        double g = matrix[6], h = matrix[7], i = matrix[8];

        double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if (fabs(det) < 1e-6)
            return false;

        double invDet = 1.0f / det;

        result[0] = (e * i - f * h) * invDet;
        result[1] = (c * h - b * i) * invDet;
        result[2] = (b * f - c * e) * invDet;
        result[3] = (f * g - d * i) * invDet;
        result[4] = (a * i - c * g) * invDet;
        result[5] = (c * d - a * f) * invDet;
        result[6] = (d * h - e * g) * invDet;
        result[7] = (b * g - a * h) * invDet;
        result[8] = (a * e - b * d) * invDet;

        return true;
    }

    // For matrices 4x4 and larger, use Gauss-Jordan elimination
    // with partial pivoting for numerical stability
    
    // Allocate augmented matrix [A|I] of size n x 2n
    double* aug = new double[n * 2 * n];
    if (!aug)
        return false;
    
    // Initialize augmented matrix: [A|I]
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            aug[row * 2 * n + col] = matrix[row * n + col];
            aug[row * 2 * n + (col + n)] = (row == col) ? 1.0f : 0.0f;
        }
    }
    
    // Gauss-Jordan elimination with partial pivoting
    for (int col = 0; col < n; col++) {
        // Find pivot (row with maximum absolute value in current column)
        int maxRow = col;
        double maxVal = fabs(aug[col * 2 * n + col]);
        for (int row = col + 1; row < n; row++) {
            double val = fabs(aug[row * 2 * n + col]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = row;
            }
        }
        
        // Check for singular matrix
        if (maxVal < 1e-6f) {
            delete[] aug;
            return false;
        }
        
        // Swap rows if needed
        if (maxRow != col) {
            for (int k = 0; k < 2 * n; k++) {
                double tmp = aug[col * 2 * n + k];
                aug[col * 2 * n + k] = aug[maxRow * 2 * n + k];
                aug[maxRow * 2 * n + k] = tmp;
            }
        }
        
        // Scale pivot row to make pivot element 1
        double pivot = aug[col * 2 * n + col];
        for (int k = 0; k < 2 * n; k++) {
            aug[col * 2 * n + k] /= pivot;
        }
        
        // Eliminate column in all other rows
        for (int row = 0; row < n; row++) {
            if (row != col) {
                double factor = aug[row * 2 * n + col];
                for (int k = 0; k < 2 * n; k++) {
                    aug[row * 2 * n + k] -= factor * aug[col * 2 * n + k];
                }
            }
        }
    }
    
    // Extract inverse from right side of augmented matrix
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            result[row * n + col] = aug[row * 2 * n + (col + n)];
        }
    }
    
    delete[] aug;
    return true;
}

bool AutoLQR::isSystemControllable()
{
    // Use full controllability matrix check
    int rank = getControllabilityRank();
    return rank == stateSize;
}

bool AutoLQR::getControllabilityMatrix(double* C)
{
    if (!C || !A || !B)
        return false;
    
    buildControllabilityMatrix(C);
    return true;
}

int AutoLQR::getControllabilityRank()
{
    if (!A || !B || stateSize <= 0 || controlSize <= 0)
        return -1;
    
    // Controllability matrix size: stateSize x (stateSize * controlSize)
    int cols = stateSize * controlSize;
    double* C = new double[stateSize * cols]();
    
    buildControllabilityMatrix(C);
    
    // Compute rank using Gaussian elimination
    int rank = computeMatrixRank(C, stateSize, cols);
    
    delete[] C;
    return rank;
}

int AutoLQR::getStateSize() const
{
    return stateSize;
}

int AutoLQR::getControlSize() const
{
    return controlSize;
}

const double* AutoLQR::getRicattiSolution() const
{
    return P;
}

void AutoLQR::buildControllabilityMatrix(double* C)
{
    // Build controllability matrix C = [B, AB, A²B, ..., A^(n-1)B]
    // C is stateSize x (stateSize * controlSize)
    
    int cols = stateSize * controlSize;
    
    // First block: B
    for (int i = 0; i < stateSize; i++) {
        for (int j = 0; j < controlSize; j++) {
            C[i * cols + j] = B[i * controlSize + j];
        }
    }
    
    // Subsequent blocks: A^k * B for k = 1, 2, ..., stateSize-1
    double* A_power = new double[stateSize * stateSize]();
    double* temp = new double[stateSize * controlSize]();
    
    // Initialize A_power as A
    memcpy(A_power, A, sizeof(double) * stateSize * stateSize);
    
    for (int k = 1; k < stateSize; k++) {
        // Compute A^k * B
        matrixMultiply(A_power, B, temp, stateSize, stateSize, controlSize);
        
        // Copy to controllability matrix
        int colOffset = k * controlSize;
        for (int i = 0; i < stateSize; i++) {
            for (int j = 0; j < controlSize; j++) {
                C[i * cols + colOffset + j] = temp[i * controlSize + j];
            }
        }
        
        // Update A_power = A_power * A for next iteration
        double* A_new = new double[stateSize * stateSize]();
        matrixMultiply(A_power, A, A_new, stateSize, stateSize, stateSize);
        memcpy(A_power, A_new, sizeof(double) * stateSize * stateSize);
        delete[] A_new;
    }
    
    delete[] A_power;
    delete[] temp;
}

int AutoLQR::computeMatrixRank(double* matrix, int rows, int cols)
{
    // Gaussian elimination with partial pivoting to find rank
    // Works on a copy to preserve original matrix
    
    double* M = new double[rows * cols];
    memcpy(M, matrix, sizeof(double) * rows * cols);
    
    const double epsilon = 1e-6f;
    int rank = 0;
    int pivotCol = 0;
    
    for (int row = 0; row < rows && pivotCol < cols; row++) {
        // Find pivot
        int maxRow = row;
        double maxVal = fabs(M[row * cols + pivotCol]);
        
        for (int i = row + 1; i < rows; i++) {
            double val = fabs(M[i * cols + pivotCol]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = i;
            }
        }
        
        if (maxVal < epsilon) {
            // No pivot in this column, move to next column
            pivotCol++;
            row--;  // Stay on same row
            continue;
        }
        
        // Swap rows
        if (maxRow != row) {
            for (int j = 0; j < cols; j++) {
                double temp = M[row * cols + j];
                M[row * cols + j] = M[maxRow * cols + j];
                M[maxRow * cols + j] = temp;
            }
        }
        
        // Eliminate column
        for (int i = row + 1; i < rows; i++) {
            double factor = M[i * cols + pivotCol] / M[row * cols + pivotCol];
            for (int j = pivotCol; j < cols; j++) {
                M[i * cols + j] -= factor * M[row * cols + j];
            }
        }
        
        rank++;
        pivotCol++;
    }
    
    delete[] M;
    return rank;
}

double AutoLQR::computeDeterminant(const double* matrix, int n)
{
    if (n == 1) {
        return matrix[0];
    } else if (n == 2) {
        return matrix[0] * matrix[3] - matrix[1] * matrix[2];
    } else if (n == 3) {
        double a = matrix[0], b = matrix[1], c = matrix[2];
        double d = matrix[3], e = matrix[4], f = matrix[5];
        double g = matrix[6], h = matrix[7], i = matrix[8];
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    } else if (n == 4) {
        // 4x4 determinant using cofactor expansion
        double det = 0;
        double submatrix[9];
        
        for (int col = 0; col < 4; col++) {
            // Build 3x3 submatrix
            int subi = 0;
            for (int i = 1; i < 4; i++) {
                int subj = 0;
                for (int j = 0; j < 4; j++) {
                    if (j == col) continue;
                    submatrix[subi * 3 + subj] = matrix[i * 4 + j];
                    subj++;
                }
                subi++;
            }
            
            double sign = (col % 2 == 0) ? 1.0f : -1.0f;
            det += sign * matrix[col] * computeDeterminant(submatrix, 3);
        }
        
        return det;
    }
    
    return 0.0f;
}

bool AutoLQR::computeGainMatrix()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Check if system is controllable
    if (!isSystemControllable()) {
        //return false;
    }

    // Iterative DARE solver for small systems
    double* P_next = new double[stateSize * stateSize]();
    double* BT = new double[controlSize * stateSize]();
    double* AT = new double[stateSize * stateSize]();
    double* temp1 = new double[controlSize * stateSize]();
    double* temp2 = new double[controlSize * controlSize]();
    double* temp3 = new double[controlSize * stateSize]();
    double* temp4 = new double[stateSize * stateSize]();
    double* temp5 = new double[stateSize * stateSize]();

    // Initialize P as Q
    memcpy(P, Q, sizeof(double) * stateSize * stateSize);

    // Compute B transpose and A transpose
    transposeMatrix(B, BT, stateSize, controlSize);
    transposeMatrix(A, AT, stateSize, stateSize);

    // Iterate to solve DARE: P = A'PA - A'PB(R + B'PB)^(-1)B'PA + Q
    for (int iter = 0; iter < solverMaxIterations; iter++) {
        // temp1 = B'P
        matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);

        // temp2 = B'PB
        matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

        // temp2 = R + B'PB
        for (int i = 0; i < controlSize * controlSize; i++) {
            temp2[i] += R[i];
        }

        // Invert (R + B'PB)
        if (!invertMatrix(temp2, temp2, controlSize)) {
            // Clean up memory
            delete[] P_next;
            delete[] BT;
            delete[] AT;
            delete[] temp1;
            delete[] temp2;
            delete[] temp3;
            delete[] temp4;
            delete[] temp5;
            return false;
        }

        // temp3 = (R + B'PB)^(-1) * B'P
        matrixMultiply(temp2, temp1, temp3, controlSize, controlSize, stateSize);

        // temp4 = A'PA
        matrixMultiply(AT, P, temp4, stateSize, stateSize, stateSize);
        matrixMultiply(temp4, A, P_next, stateSize, stateSize, stateSize);

        // temp5 = A'PB
        matrixMultiply(AT, P, temp4, stateSize, stateSize, stateSize);
        matrixMultiply(temp4, B, temp5, stateSize, stateSize, controlSize);

        // temp4 = A'PB * (R + B'PB)^(-1) * B'PA
        matrixMultiply(temp5, temp3, temp4, stateSize, controlSize, stateSize);
        matrixMultiply(temp4, A, temp5, stateSize, stateSize, stateSize);

        // P_next = A'PA - A'PB * (R + B'PB)^(-1) * B'PA + Q
        for (int i = 0; i < stateSize * stateSize; i++) {
            P_next[i] = P_next[i] - temp5[i] + Q[i];
        }

        // Check convergence
        double diff = 0;
        for (int i = 0; i < stateSize * stateSize; i++) {
            diff += fabs(P_next[i] - P[i]);
        }

        // Update P for next iteration
        memcpy(P, P_next, sizeof(double) * stateSize * stateSize);

        if (diff < solverTolerance) {
            break; // Converged
        }
    }

    // Compute K = (R + B'PB)^(-1) * B'PA
    matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);
    matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

    for (int i = 0; i < controlSize * controlSize; i++) {
        temp2[i] += R[i];
    }

    if (!invertMatrix(temp2, temp2, controlSize)) {
        // Clean up memory
        delete[] P_next;
        delete[] BT;
        delete[] AT;
        delete[] temp1;
        delete[] temp2;
        delete[] temp3;
        delete[] temp4;
        delete[] temp5;
        return false;
    }

    matrixMultiply(temp1, A, temp3, controlSize, stateSize, stateSize);
    matrixMultiply(temp2, temp3, K, controlSize, controlSize, stateSize);

    // Clean up memory
    delete[] P_next;
    delete[] BT;
    delete[] AT;
    delete[] temp1;
    delete[] temp2;
    delete[] temp3;
    delete[] temp4;
    delete[] temp5;

    return true;
}

void AutoLQR::estimateFeedforwardGain(double* ffGain, const double* desiredState)
{
    if (!ffGain || !desiredState || !A || !B || !K)
        return;

    // For steady-state tracking: u_ff = -(inv(B'B) * B' * (A-I)) * x_desired
    // This is simplified for common cases

    if (stateSize == 2 && controlSize == 1) {
        // Special case for position-velocity systems
        double Bsq = B[0] * B[0] + B[1] * B[1];
        if (Bsq < 1e-6)
            return;

        double invBsq = 1.0f / Bsq;

        // Compute (A-I) * x_desired
        double dx[2];
        dx[0] = (A[0] - 1.0f) * desiredState[0] + A[1] * desiredState[1];
        dx[1] = A[2] * desiredState[0] + (A[3] - 1.0f) * desiredState[1];

        // Compute feedforward gain
        ffGain[0] = -invBsq * (B[0] * dx[0] + B[1] * dx[1]);
    } else {
        // For other systems, initialize to zero
        for (int i = 0; i < controlSize; i++) {
            ffGain[i] = 0;
        }
    }
}

double AutoLQR::estimateConvergenceTime(double convergenceThreshold)
{
    // Estimate convergence time based on eigenvalues
    // This is a simplified estimate for 2x2 systems

    if (stateSize == 2) {
        // Compute closed-loop dynamics: A - B*K
        double CL[4];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                CL[i * 2 + j] = A[i * 2 + j];
                for (int k = 0; k < controlSize; k++) {
                    CL[i * 2 + j] -= B[i * controlSize + k] * K[k * stateSize + j];
                }
            }
        }

        // Approximate dominant eigenvalue using trace and determinant
        double trace = CL[0] + CL[3];
        double det = CL[0] * CL[3] - CL[1] * CL[2];

        // Characteristic equation: λ² - trace·λ + det = 0
        double discriminant = trace * trace - 4 * det;

        if (discriminant >= 0) {
            // Real eigenvalues
            double lambda1 = (trace + sqrt(discriminant)) / 2;
            double lambda2 = (trace - sqrt(discriminant)) / 2;

            // Dominant eigenvalue (larger magnitude)
            double domEigenvalue = (fabs(lambda1) > fabs(lambda2)) ? lambda1 : lambda2;

            if (fabs(domEigenvalue) < 1.0f && fabs(domEigenvalue) > 0.0f) {
                // Estimate time constant
                double timeConstant = -1.0f / log(fabs(domEigenvalue));

                // Time to reach convergenceThreshold
                return timeConstant * log(1.0f / convergenceThreshold);
            }
        }
    }

    // Default value if calculation fails
    return -1.0f;
}

bool AutoLQR::exportGains(double* exportedK)
{
    if (!K || !exportedK)
        return false;

    memcpy(exportedK, K, sizeof(double) * controlSize * stateSize);
    return true;
}

double AutoLQR::calculateExpectedCost()
{
    if (!P || !state)
        return -1.0f;

    // Cost = x'Px
    double cost = 0;
    for (int i = 0; i < stateSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            cost += state[i] * P[i * stateSize + j] * state[j];
        }
    }

    return cost;
}
