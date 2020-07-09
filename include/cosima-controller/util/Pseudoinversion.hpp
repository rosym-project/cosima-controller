#include <Eigen/Dense>
#pragma once

namespace cosima
{
template <typename Derived>
class Pseudoinverse
{
  public:
    Pseudoinverse(Derived const &input, Derived const &weighting, float thresh = 1e-4) : Pseudoinverse(input, thresh)
    {
        chol = Eigen::LLT<Derived>(weighting);
        L = Eigen::MatrixXf::Zero(weighting.rows(), weighting.cols());
    }
    Pseudoinverse(Derived const &input, float thresh = 1e-4) : qr(input), qr_nullspace(input.transpose())
    {
        qr.setThreshold(thresh);
        qr_nullspace.setThreshold(thresh);
        this->input = input;
        pinv.resize(input.cols(), input.rows());
        pinv.setZero();
        R1.resize(input.rows(), input.cols());
        R1.setZero();
        ginv.resize(input.cols(), input.rows());
        ginv.setZero();
        R3.resize(input.cols(), input.rows());
        R3.setZero();
        null.resize(input.cols(), input.cols());
        nullspace_eye = Derived::Identity(input.cols(), input.cols());
        activation_matrix = Derived::Zero(input.rows(), input.rows());
        Q.resize(input.cols(), input.rows());
        Q.setZero();
    }
    Eigen::Matrix<typename Derived::Scalar, -1, -1> &compute(Derived const &input, float thresh = 1e-4)
    {
        qr.setThreshold(thresh);
        qr.compute(input);
        rank = qr.rank();
        if (rank == 0)
        {
            pinv.setZero();
            return pinv;
        }
        cols = qr.matrixR().cols();
        R1.block(0, 0, rank, cols) = qr.matrixR().topLeftCorner(rank, qr.matrixR().cols()).template triangularView<Eigen::Upper>();
        if (rank > qr.matrixR().cols())
        {
            ginv.block(0, 0, cols, rank) = (R1.block(0, 0, rank, cols).transpose() * R1.block(0, 0, rank, cols)).inverse() * R1.block(0, 0, rank, cols).transpose();
        }
        else
        {
            ginv.block(0, 0, cols, rank) = ((R1.block(0, 0, rank, cols) * R1.block(0, 0, rank, cols).transpose()).inverse() * R1.block(0, 0, rank, cols)).transpose();
        }
        R3.block(0, 0, input.cols(), rank) = ginv.block(0, 0, cols, rank);
        pinv = qr.colsPermutation() * R3 * qr.householderQ().transpose();
        return pinv;
    }

    Eigen::Matrix<typename Derived::Scalar, -1, -1> &compute(Derived const &input, Eigen::Matrix<typename Derived::Scalar, -1, 1> const &activation, float thresh = 1e-4)
    {
        qr.setThreshold(thresh);
        qr.compute(input);
        rank = qr.rank();
        if (rank == 0)
        {
            pinv.setZero();
            return pinv;
        }
        cols = qr.matrixR().cols();
        R1.block(0, 0, rank, cols) = qr.matrixR().topLeftCorner(rank, qr.matrixR().cols()).template triangularView<Eigen::Upper>();
        if (rank > qr.matrixR().cols())
        {
            ginv.block(0, 0, cols, rank) = (R1.block(0, 0, rank, cols).transpose() * R1.block(0, 0, rank, cols)).inverse() * R1.block(0, 0, rank, cols).transpose();
        }
        else
        {
            ginv.block(0, 0, cols, rank) = ((R1.block(0, 0, rank, cols) * R1.block(0, 0, rank, cols).transpose()).inverse() * R1.block(0, 0, rank, cols)).transpose();
        }
        R3.block(0, 0, input.cols(), rank) = ginv.block(0, 0, cols, rank);
        activation_matrix.setZero();
        activation_matrix.block(0, 0, qr.rank(), qr.rank()).setIdentity();
        activation_matrix *= activation.asDiagonal();
        pinv = activation_matrix * qr.colsPermutation() * R3 * qr.householderQ().transpose();
        return pinv;
    }

    Eigen::Matrix<typename Derived::Scalar, -1, -1> &calculateNullspace(Derived const &input)
    {
        qr.compute(input.transpose());
        activation_matrix.setZero();
        activation_matrix.block(0, 0, qr.rank(), qr.rank()).setIdentity();
        // std::cout<<"MATRIX TMP: "<<(chol.matrixL() * (qr.colsPermutation() * R3 * qr.householderQ().transpose()))<<"\n";
        // std::cout<<"MATRIX L: "<<(chol.matrixL().cols())<< ", "<< (chol.matrixL().rows())<<"\n";
        // std::cout<<"MATRIX TMP: "<<(qr.colsPermutation() * R3 * qr.householderQ().transpose())<<"\n";
        Q.setIdentity();

        Q = qr.householderQ().setLength(qr.nonzeroPivots()) * Q;
        null = nullspace_eye - Q * activation_matrix * Q.transpose();
        // std::cout<<"MATRIX EYE: \n"<<eye<<"\n";
        // std::cout<<"MATRIX RIGHT: \n"<<right<<"\n";
        // std::cout<<"MATRIX EYE-RIGHT: \n"<<eye-right<<"\n";
        // std::cout<<"MATRIX Null: \n"<<null<<"\n";
        return null;
    }
    Eigen::Matrix<typename Derived::Scalar, -1, -1> &calculateNullspace(Derived const &input, Eigen::Matrix<typename Derived::Scalar, -1, 1> const &activation)
    {
        qr.compute(input.transpose());
        activation_matrix.setZero();
        activation_matrix.block(0, 0, qr.rank(), qr.rank()).setIdentity();
        activation_matrix *= activation.asDiagonal();
        // std::cout<<"MATRIX TMP: "<<(chol.matrixL() * (qr.colsPermutation() * R3 * qr.householderQ().transpose()))<<"\n";
        // std::cout<<"MATRIX L: "<<(chol.matrixL().cols())<< ", "<< (chol.matrixL().rows())<<"\n";
        // std::cout<<"MATRIX TMP: "<<(qr.colsPermutation() * R3 * qr.householderQ().transpose())<<"\n";
        Q.setIdentity();
        Q = qr.householderQ().setLength(qr.nonzeroPivots()) * Q;
        null = nullspace_eye - Q * activation_matrix * Q.transpose();
        // std::cout<<"MATRIX EYE: \n"<<nullspace_eye<<"\n";
        // std::cout<<"MATRIX RIGHT: \n"<<right<<"\n";
        // std::cout<<"MATRIX EYE-RIGHT: \n"<<nullspace_eye-right<<"\n";
        // std::cout<<"MATRIX Null: \n"<<null<<"\n";
        return null;
    }

    Eigen::Matrix<typename Derived::Scalar, -1, -1> &compute(Derived const &input, Derived const &weighting)
    {
        chol.compute(weighting);
        qr.compute(input * chol.matrixL());
        rank = qr.rank();
        if (rank == 0)
        {
            pinv.setZero();
            return pinv;
        }
        cols = qr.matrixR().cols();
        R1.block(0, 0, rank, cols) = qr.matrixR().topLeftCorner(rank, qr.matrixR().cols()).template triangularView<Eigen::Upper>();
        if (rank > qr.matrixR().cols())
        {
            ginv.block(0, 0, cols, rank) = (R1.block(0, 0, rank, cols).transpose() * R1.block(0, 0, rank, cols)).inverse() * R1.block(0, 0, rank, cols).transpose();
        }
        else
        {
            ginv.block(0, 0, cols, rank) = ((R1.block(0, 0, rank, cols) * R1.block(0, 0, rank, cols).transpose()).inverse() * R1.block(0, 0, rank, cols)).transpose();
        }
        R3.block(0, 0, input.cols(), rank) = ginv.block(0, 0, cols, rank);
        pinv = chol.matrixL() * (qr.colsPermutation() * R3 * qr.householderQ().transpose());
        return pinv;
    }
    Eigen::Matrix<typename Derived::Scalar, -1, -1> &calculateNullspace(Derived const &input, Derived const &weighting)
    {
        chol.compute(weighting);
        //std::cout<<"input.t size "<<(input * chol.matrixL()).transpose().rows()<<", "<<(input * chol.matrixL()).transpose().cols()<<"\n";
        qr.compute((input * chol.matrixL()).transpose());
        rank = qr.rank();
        L = chol.matrixL();
        //std::cout<<"activation matrix:"<<activation_matrix.rows()<<", "<<activation_matrix.cols()<<"\n";
        activation_matrix.resize(input.cols(), input.cols());
        activation_matrix.setZero();
        activation_matrix.block(0, 0, qr.rank(), qr.rank()).setIdentity();
        //std::cout<<"activation matrix:"<<activation_matrix<<"\n";
        // std::cout<<"MATRIX TMP: "<<(chol.matrixL() * (qr.colsPermutation() * R3 * qr.householderQ().transpose()))<<"\n";
        // std::cout<<"MATRIX L: "<<(chol.matrixL().cols())<< ", "<< (chol.matrixL().rows())<<"\n";
        // std::cout<<"MATRIX TMP: "<<(qr.colsPermutation() * R3 * qr.householderQ().transpose())<<"\n";
        Q.setIdentity();
        //std::cout<<"Q:"<<Q<<"\n";
        Q = qr.householderQ().setLength(qr.nonzeroPivots()) * Q;
        null = nullspace_eye - (L.transpose().inverse() * (Q * activation_matrix * Q.transpose()) * L.transpose());
        // std::cout<<"MATRIX EYE: \n"<<eye<<"\n";
        // std::cout<<"MATRIX RIGHT: \n"<<right<<"\n";
        // std::cout<<"MATRIX EYE-RIGHT: \n"<<eye-right<<"\n";
        // std::cout<<"MATRIX Null: \n"<<null<<"\n";
        return null;
    }

    Eigen::Matrix<typename Derived::Scalar, -1, -1> &calculateNullspace(Derived const &input, Derived const &weighting, Eigen::Matrix<typename Derived::Scalar, -1, 1> const &activation)
    {
        chol.compute(weighting);
        qr.compute((input * chol.matrixL()).transpose());

        L = chol.matrixL();
        activation_matrix.setZero();
        activation_matrix.block(0, 0, qr.rank(), qr.rank()).setIdentity();
        activation_matrix *= activation.asDiagonal();
        // std::cout<<"MATRIX TMP: "<<(chol.matrixL() * (qr.colsPermutation() * R3 * qr.householderQ().transpose()))<<"\n";
        // std::cout<<"MATRIX L: "<<(chol.matrixL().cols())<< ", "<< (chol.matrixL().rows())<<"\n";
        // std::cout<<"MATRIX TMP: "<<(qr.colsPermutation() * R3 * qr.householderQ().transpose())<<"\n";
        Q.setIdentity();
        Q = qr.householderQ().setLength(qr.nonzeroPivots()) * Q;
        null = nullspace_eye - (L.inverse() * (Q * activation_matrix * Q.transpose()) * L);
        // std::cout<<"MATRIX EYE: \n"<<eye<<"\n";
        // std::cout<<"MATRIX RIGHT: \n"<<right<<"\n";
        // std::cout<<"MATRIX EYE-RIGHT: \n"<<eye-right<<"\n";
        // std::cout<<"MATRIX Null: \n"<<null<<"\n";
        return null;
    }
    int getRank()
    {
        return rank;
    }

  private:
    Eigen::ColPivHouseholderQR<Derived> qr, qr_nullspace;
    Eigen::LLT<Derived> chol;
    Eigen::Matrix<typename Derived::Scalar, -1, -1> R1;
    Eigen::Matrix<typename Derived::Scalar, -1, -1> ginv;
    Eigen::Matrix<typename Derived::Scalar, -1, -1> R3;
    Eigen::Matrix<typename Derived::Scalar, -1, -1> pinv;
    Eigen::Matrix<typename Derived::Scalar, -1, -1> null;
    Derived input;
    int rank, cols;
    bool weighted = false;
    Derived nullspace_eye;
    Derived activation_matrix;
    Eigen::MatrixXf L, Q;
};
}; // namespace cosima
