/** Author: Niels Dehio
 * Date:   30 May 2017
 *
 * Description:
 */

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Core> //TODO necessary?
#include <math.h>     // isinf, sqrt

using namespace Eigen; //necessary because single code line containing template triangularView,
                       //see also https://eigen.tuxfamily.org/dox/TopicTemplateKeyword.html

namespace cosima
{

namespace util
{

//template<typename mytype>
class MatrixDecomposition {

public:

MatrixDecomposition() {
    svdJacobi = Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >();
    svdBDC = Eigen::BDCSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >();
    qrDecomposition = Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >();
    qrDecompositionCol = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >();
    chol = Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >();
    cholPivot = Eigen::LDLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >();

    borderA = 0.1;
    borderB = 0.01;
    epsilon = std::numeric_limits<double>::min();
    changeRows = true;
    useAauto = true;
    useAuser = true;
}

void output(){
    std::cout << "output" << std::endl;
    std::cout << "borderA = " << borderA << std::endl;
    std::cout << "borderB = " << borderB << std::endl;
    std::cout << "epsilon = " << epsilon << std::endl;

}

//#############################################


void computeDLSpseudoinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                         double const reg,
                         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(pseudoinv.rows() == cols);
    assert(pseudoinv.cols() == rows);
    assert(reg>=0.0);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DLS;
    DLS = reg * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(rows,rows);

    pseudoinv = Mat.transpose() * (Mat * Mat.transpose() + DLS).inverse();
}


void computeDLSpseudoinvWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                 double const reg,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);
    assert(pseudoinv.rows() == cols);
    assert(pseudoinv.cols() == rows);
    assert(reg>=0.0);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DLS;
    DLS = reg * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(rows,rows);

    //version1
    pseudoinv = W * Mat.transpose() * (Mat * W * Mat.transpose() + DLS).inverse();

    //version2
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
//    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
//    this->computeCHOL(W, L);
//    //pseudoinv = L * (Mat*L).transpose() * ((Mat*L) * (Mat*L).transpose() + DLS).inverse();
//    this->computeDLSpseudoinv(Mat*L, reg, pseudoinv);
//    pseudoinv = L * pseudoinv;
}



void computeSVDpseudoinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv){
    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeSVDpseudoinv(Mat, AsvdUser, pseudoinv, rank, AsvdAuto);
}

void computeSVDpseudoinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                         Eigen::Matrix<double, Eigen::Dynamic, 1> const & AsvdUser,
                         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv,
                         int & rank,
                         Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(AsvdUser.rows() == rows);
    assert(AsvdUser.cols() == 1);
    assert(pseudoinv.rows() == cols);
    assert(pseudoinv.cols() == rows);
    assert(AsvdAuto.rows() == rows);
    assert(AsvdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V, U;
    Eigen::Matrix<double, Eigen::Dynamic, 1> S, Spinv;
    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
    S = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);
    Spinv = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

    this->computeSVD(Mat, U, S, V, rank);
    this->computeSpinv(S, Spinv);
    this->computeSVDActivationMat(S, AsvdAuto);
    //this->checkOrthogonality(V, 0.01);

    Eigen::Matrix<double, Eigen::Dynamic, 1> Asvd = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->combineActivation(AsvdAuto, AsvdUser, Asvd);
    pseudoinv = V * Spinv.asDiagonal() * Asvd.asDiagonal() * U.transpose();
}



void computeSVDpseudoinvWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv){
    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeSVDpseudoinvWeighted(Mat, W, AsvdUser, pseudoinv, rank, AsvdAuto);
}

void computeSVDpseudoinvWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1> const & AsvdUser,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv,
                                 int & rank,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);
    assert(AsvdUser.rows() == rows);
    assert(AsvdUser.cols() == 1);
    assert(pseudoinv.rows() == cols);
    assert(pseudoinv.cols() == rows);
    assert(AsvdAuto.rows() == rows);
    assert(AsvdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
    this->computeCHOL(W, L);

    this->computeSVDpseudoinv(Mat*L, AsvdUser, pseudoinv, rank, AsvdAuto);
    pseudoinv = L * pseudoinv;
}



void computeQRDpseudoinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv){
    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeQRDpseudoinv(Mat, AqrdUser, pseudoinv, rank, AqrdAuto);
}

void computeQRDpseudoinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                        Eigen::Matrix<double, Eigen::Dynamic, 1> const & AqrdUser,
                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv,
                        int & rank,
                        Eigen::Matrix<double, Eigen::Dynamic, 1> & AqrdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(AqrdUser.rows() == rows);
    assert(AqrdUser.cols() == 1);
    assert(pseudoinv.rows() == cols);
    assert(pseudoinv.cols() == rows);
    assert(AqrdAuto.rows() == rows);
    assert(AqrdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q, R;
    Eigen::VectorXi Pvec;
    Eigen::MatrixXi Pmat;
    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
    Pvec = Eigen::VectorXi::Zero(rows);
    Pmat = Eigen::MatrixXi::Zero(rows,rows);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RTpinv_right
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);

    //this->computeQRD(Mat.transpose(), Q, R, rank, Pvec);
    this->computeQRD2(Mat.transpose(), Q, R, rank, Pvec, false);
    this->permutationVec2Mat(Pvec, Pmat);
    this->computeRpinv(R, rank, false, RTpinv_right);
    this->computeQRDActivationMat(R, AqrdAuto);
    //this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...

    Eigen::Matrix<double, Eigen::Dynamic, 1> Aqrd = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->combineActivation(AqrdAuto, AqrdUser, Aqrd);
    pseudoinv = Q * RTpinv_right * Aqrd.asDiagonal() * Pmat.transpose().cast<double>();
}



void computeQRDpseudoinvWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv){
    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeQRDpseudoinvWeighted(Mat, W, AqrdUser, pseudoinv, rank, AqrdAuto);
}

void computeQRDpseudoinvWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                Eigen::Matrix<double, Eigen::Dynamic, 1> const & AqrdUser,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv,
                                int & rank,
                                Eigen::Matrix<double, Eigen::Dynamic, 1> & AqrdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);
    assert(AqrdUser.rows() == rows);
    assert(AqrdUser.cols() == 1);
    assert(pseudoinv.rows() == cols);
    assert(pseudoinv.cols() == rows);
    assert(AqrdAuto.rows() == rows);
    assert(AqrdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
    this->computeCHOL(W, L);

    this->computeQRDpseudoinv(Mat*L, AqrdUser, pseudoinv, rank, AqrdAuto);
    pseudoinv = L * pseudoinv;
}



//void computeSVDQRpseudoinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv, int & rankSVD, int & rankQRD){
//    //expect Mat=Jacobian

//    unsigned int rows, cols;
//    rows = Mat.rows();
//    cols = Mat.cols();

//    assert(rows > 0);
//    assert(cols > 0);
//    assert(pseudoinv.rows() == cols);
//    assert(pseudoinv.cols() == rows);

//    //QRD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q, R;
//    Eigen::VectorXi Pvec;
//    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
//    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
//    Pvec = Eigen::VectorXi::Zero(rows);

//    //this->computeQRD(Mat.transpose(), Q, R, rankQRD, Pvec);
//    this->computeQRD2(Mat.transpose(), Q, R, rankQRD, Pvec, false);
//    this->computeQRDActivationMat(R, AqrdAuto);
//    //this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...

//    //SVD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V, U;
//    Eigen::Matrix<double, Eigen::Dynamic, 1> S, Spinv;
//    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
//    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
//    S = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);
//    Spinv = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

//    this->computeSVD(Mat, U, S, V, rankSVD);
//    this->computeSpinv(S, Spinv);
//    this->computeSVDActivationMat(S, AsvdAuto);
//    //this->checkOrthogonality(V, 0.01);

//    //SVDQR
//    Eigen::Matrix<double, Eigen::Dynamic, 1> CT;
//    CT = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);
//    for(unsigned int row=0; row<rows; row++){
//        CT(row) = sqrt(AsvdAuto(row));
//    }

//    pseudoinv = V * CT.asDiagonal() * Spinv.asDiagonal() * U.transpose() * R.transpose();
//}



//void computeSVDQRpseudoinvWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & pseudoinv, int & rankSVD, int & rankQRD){
//    //expect Mat=Jacobian

//    unsigned int rows, cols;
//    rows = Mat.rows();
//    cols = Mat.cols();

//    assert(rows > 0);
//    assert(cols > 0);
//    assert(W.rows() == cols);
//    assert(W.cols() == cols);
//    assert(pseudoinv.rows() == cols);
//    assert(pseudoinv.cols() == rows);

//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
//    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
//    this->computeCHOL(W, L);

//    this->computeSVDQRpseudoinv(Mat*L, pseudoinv, rankSVD, rankQRD);
//    pseudoinv = L * pseudoinv;
//}

//#############################################


void computeDLSnullspace(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                         double const reg,
                         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(nullspace.rows() == cols);
    assert(nullspace.cols() == cols);
    assert(reg>=0.0);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DLS = reg * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(rows,rows);

    nullspace = eye - (Mat.transpose() * (Mat * Mat.transpose() + DLS).inverse() * Mat);
}



void computeDLSnullspaceWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                 double const reg,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);
    assert(nullspace.rows() == cols);
    assert(nullspace.cols() == cols);
    assert(reg>=0.0);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DLS = reg * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(rows,rows);

    //version1
    nullspace = eye - (Mat.transpose() * (Mat * W * Mat.transpose() + DLS).inverse() * Mat * W);
    //std::cout << "nullspace1 = " << nullspace << std::endl;

    //version2
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
//    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
//    this->computeCHOL(W, L);
//    nullspace = eye - (L.transpose().inverse() * (Mat*L).transpose() * ((Mat*L) * (Mat*L).transpose() + DLS).inverse() * (Mat*L) * L.transpose());
//    std::cout << "nullspace2 = " << nullspace << std::endl;
}



void computeSVDnullspace(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace){

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeSVDnullspace(Mat, AsvdUser, nullspace, rank, AsvdAuto);
}

void computeSVDnullspace(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                         Eigen::Matrix<double, Eigen::Dynamic, 1> const & AsvdUser,
                         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace,
                         int & rank,
                         Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(AsvdUser.rows() == rows);
    assert(AsvdUser.cols() == 1);
    assert(nullspace.rows() == cols);
    assert(nullspace.cols() == cols);
    assert(AsvdAuto.rows() == rows);
    assert(AsvdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye, V, U;
    Eigen::Matrix<double, Eigen::Dynamic, 1> S;
    eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
    S = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

    this->computeSVD(Mat, U, S, V, rank);
    this->computeSVDActivationMat(S, AsvdAuto);
    //this->checkOrthogonality(V, 0.01);

    Eigen::Matrix<double, Eigen::Dynamic, 1> Asvd = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->combineActivation(AsvdAuto, AsvdUser, Asvd);
    nullspace = eye - V * Asvd.asDiagonal() * V.transpose();
}



void computeSVDnullspaceWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace){

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeSVDnullspaceWeighted(Mat, W, AsvdUser, nullspace, rank, AsvdAuto);
}

void computeSVDnullspaceWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1> const & AsvdUser,
                                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace,
                                 int & rank,
                                 Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);
    assert(AsvdUser.rows() == rows);
    assert(AsvdUser.cols() == 1);
    assert(nullspace.rows() == cols);
    assert(nullspace.cols() == cols);
    assert(AsvdAuto.rows() == rows);
    assert(AsvdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
    this->computeCHOL(W, L);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye, V, U;
    Eigen::Matrix<double, Eigen::Dynamic, 1> S;
    eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
    S = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

    this->computeSVD(Mat*L, U, S, V, rank);
    this->computeSVDActivationMat(S, AsvdAuto);
    //this->checkOrthogonality(V, 0.01);

    Eigen::Matrix<double, Eigen::Dynamic, 1> Asvd = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->combineActivation(AsvdAuto, AsvdUser, Asvd);
    nullspace = eye - L.transpose().inverse() * V * Asvd.asDiagonal() * V.transpose() * L.transpose();
}



void computeQRDnullspace(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace){

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeQRDnullspace(Mat, AqrdUser, nullspace, rank, AqrdAuto);
}

void computeQRDnullspace(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                        Eigen::Matrix<double, Eigen::Dynamic, 1> const & AqrdUser,
                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace,
                        int & rank,
                        Eigen::Matrix<double, Eigen::Dynamic, 1> & AqrdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(AqrdUser.rows() == rows);
    assert(AqrdUser.cols() == 1);
    assert(nullspace.rows() == cols);
    assert(nullspace.cols() == cols);
    assert(AqrdAuto.rows() == rows);
    assert(AqrdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye, Q, R;
    Eigen::VectorXi Pvec;
    eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
    Pvec = Eigen::VectorXi::Zero(rows);

    //this->computeQRD(Mat.transpose(), Q, R, rank, Pvec);
    this->computeQRD2(Mat.transpose(), Q, R, rank, Pvec, false);
    this->computeQRDActivationMat(R, AqrdAuto);
    //this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...

    Eigen::Matrix<double, Eigen::Dynamic, 1> Aqrd = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->combineActivation(AqrdAuto, AqrdUser, Aqrd);
    nullspace = eye - Q * Aqrd.asDiagonal() * Q.transpose();
}



void computeQRDnullspaceWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace){

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    int rank;
    Eigen::Matrix<double, Eigen::Dynamic, 1> AqrdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->computeQRDnullspaceWeighted(Mat, W, AqrdUser, nullspace, rank, AqrdAuto);
}

void computeQRDnullspaceWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
                                Eigen::Matrix<double, Eigen::Dynamic, 1> const & AqrdUser,
                                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace,
                                int & rank,
                                Eigen::Matrix<double, Eigen::Dynamic, 1> & AqrdAuto){
    //expect Mat=Jacobian

    unsigned int rows, cols;
    rows = Mat.rows();
    cols = Mat.cols();

    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);
    assert(AqrdUser.rows() == rows);
    assert(AqrdUser.cols() == 1);
    assert(nullspace.rows() == cols);
    assert(nullspace.cols() == cols);
    assert(AqrdAuto.rows() == rows);
    assert(AqrdAuto.cols() == 1);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
    this->computeCHOL(W, L);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye, Q, R;
    Eigen::VectorXi Pvec;
    eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
    Pvec = Eigen::VectorXi::Zero(rows);

    //this->computeQRD((Mat*L).transpose(), Q, R, rank, Pvec);
    this->computeQRD2((Mat*L).transpose(), Q, R, rank, Pvec, false);
    this->computeQRDActivationMat(R, AqrdAuto);
    //this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...

    Eigen::Matrix<double, Eigen::Dynamic, 1> Aqrd = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    this->combineActivation(AqrdAuto, AqrdUser, Aqrd);
    nullspace = eye - L.transpose().inverse() * Q * Aqrd.asDiagonal() * Q.transpose() * L.transpose();
}



//void computeSVDQRnullspace(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat, Eigen::Matrix<double, Eigen::Dynamic, 1> const & AqrdAuto, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace, int & rankSVD, int & rankQRD, Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto, Eigen::Matrix<double, Eigen::Dynamic, 1> & S){
//    //expect Mat=Jacobian

//    unsigned int rows, cols;
//    rows = Mat.rows();
//    cols = Mat.cols();

//    assert(rows > 0);
//    assert(cols > 0);
//    assert(AqrdAuto.rows() == rows);
//    assert(AqrdAuto.cols() == 1);
//    assert(nullspace.rows() == cols);
//    assert(nullspace.cols() == cols);
//    assert(AsvdAuto.rows() == rows);
//    assert(AsvdAuto.cols() == 1);

//    //QRD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q, R;
//    Eigen::VectorXi Pvec;
//    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
//    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
//    Pvec = Eigen::VectorXi::Zero(rows);

//    //this->computeQRD(Mat.transpose(), Q, R, rankQRD, Pvec);
//    this->computeQRD2(Mat.transpose(), Q, R, rankQRD, Pvec, false);
//    this->computeQRDActivationMat(R, AqrdAuto);
//    //this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...

//    //SVD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V, U;
//    Eigen::Matrix<double, Eigen::Dynamic, 1> Spinv;
//    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
//    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
//    Spinv = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);
//    AsvdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

//    this->computeSVD(Mat, U, S, V, rankSVD);
//    this->computeSpinv(S, Spinv);
//    this->computeSVDActivationMat(S, AsvdAuto);
//    //this->checkOrthogonality(V, 0.01);

//    //SVDQRD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye, RtRinv, Dleft, Dright;
//    Eigen::Matrix<double, Eigen::Dynamic, 1> CT;
//    eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
//    RtRinv = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows, rows);
//    Dleft = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols, rows);
//    Dright = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols, rows);
//    CT = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

//    this->computeRtRinv(R, Pvec, rankQRD, RtRinv);
//    for(unsigned int row=0; row<rows; row++){
//        CT(row) = sqrt(AsvdAuto(row));
//    }

//    Dleft = V * S.asDiagonal() * CT.asDiagonal() * U.transpose() * RtRinv * R.transpose();
//    Dright = V * CT.asDiagonal() * Spinv.asDiagonal() * U.transpose() * R.transpose();

//    nullspace = eye - Dleft * AqrdAuto.asDiagonal() * Dright.transpose();
//}



//void computeSVDQRnullspaceWeighted(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W, Eigen::Matrix<double, Eigen::Dynamic, 1> const & AqrdAuto, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & nullspace, int & rankSVD, int & rankQRD, Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto, Eigen::Matrix<double, Eigen::Dynamic, 1> & S){
//    //expect Mat=Jacobian

//    unsigned int rows, cols;
//    rows = Mat.rows();
//    cols = Mat.cols();

//    assert(rows > 0);
//    assert(cols > 0);
//    assert(W.rows() == cols);
//    assert(W.cols() == cols);
//    assert(AqrdAuto.rows() == rows);
//    assert(AqrdAuto.cols() == 1);
//    assert(nullspace.rows() == cols);
//    assert(nullspace.cols() == cols);
//    assert(AsvdAuto.rows() == rows);
//    assert(AsvdAuto.cols() == 1);

//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
//    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
//    this->computeCHOL(W, L);

//    //QRD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q, R;
//    Eigen::VectorXi Pvec;
//    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
//    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
//    Pvec = Eigen::VectorXi::Zero(rows);

//    //this->computeQRD((Mat*L).transpose(), Q, R, rankQRD, Pvec);
//    this->computeQRD2((Mat*L).transpose(), Q, R, rankQRD, Pvec, false);
//    this->computeQRDActivationMat(R, AqrdAuto);
//    //this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...

//    //SVD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V, U;
//    Eigen::Matrix<double, Eigen::Dynamic, 1> Spinv;
//    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
//    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);
//    Spinv = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

//    this->computeSVD(Mat*L, U, S, V, rankSVD);
//    this->computeSpinv(S, Spinv);
//    this->computeSVDActivationMat(S, AsvdAuto);
//    //this->checkOrthogonality(V, 0.01);

//    //SVDQRD
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye, RtRinv, Dleft, Dright;
//    Eigen::Matrix<double, Eigen::Dynamic, 1> CT;
//    eye = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(cols,cols);
//    RtRinv = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows, rows);
//    Dleft = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols, rows);
//    Dright = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols, rows);
//    CT = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

//    this->computeRtRinv(R, Pvec, rankQRD, RtRinv);
//    for(unsigned int row=0; row<rows; row++){
//        CT(row) = sqrt(AsvdAuto(row));
//    }

//    Dleft = V * S.asDiagonal() * CT.asDiagonal() * U.transpose() * RtRinv * R.transpose();
//    Dright = V * CT.asDiagonal() * Spinv.asDiagonal() * U.transpose() * R.transpose();

//    nullspace = eye - L.transpose().inverse() * Dleft * AqrdAuto.asDiagonal() * Dright.transpose() * L.transpose();
//}

//#############################################



void computeCHOL(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & L){

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(mRows == nCols);
    assert(L.rows() == mRows);
    assert(L.cols() == mRows);

    //decomposition of a symmetric, positive definite matrix
    L = chol.compute(Mat).matrixL(); //lower triangular matrix U
    //U = chol.compute(Mat).matrixU(); //upper triangular matrix U

    //decomposition of a positive semidefinite or negative semidefinite matrix
    //L = cholPivot.compute(Mat).matrixL();
}



void computeSVD(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & U,
                Eigen::Matrix<double, Eigen::Dynamic, 1> & S,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & V,
                int & rank){
    //expect Mat=Jacobian

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(U.rows() == mRows);
    assert(U.cols() == mRows);
    assert(S.rows() == mRows);
    assert(S.cols() == 1);
    assert(V.rows() == nCols);
    assert(V.cols() == mRows);

    if(Mat.isZero()){
        U.setZero();
        S.setZero();
        V.setZero();
        rank = 0;
        return;
    }

//    svdJacobi = Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(mRows, nCols);
//    svdBDC = Eigen::BDCSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(mRows, nCols);

    svdJacobi.compute(Mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //svdJacobi.compute(Mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //svdBDC.compute(Mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //svdBDC.compute(Mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

    //rank = svdJacobi.rank();
    //rank = svdBDC.rank();

    V = svdJacobi.matrixV();//.leftCols(Spinv.size());
    U = svdJacobi.matrixU();//.leftCols(Spinv.size());
    S = svdJacobi.singularValues();

//    std::cout << "V.rows=\n" << V.rows() << std::endl;
//    std::cout << "V.cols=\n" << V.cols() << std::endl;
//    std::cout << "U.rows=\n" << U.rows() << std::endl;
//    std::cout << "U.cols=\n" << U.cols() << std::endl;
//    std::cout << "S.rows=\n" << S.rows() << std::endl;
//    std::cout << "S.cols=\n" << S.cols() << std::endl;

    rank = 0;
    for(unsigned int i=0; i<mRows; i++){
        if(S(i) > 0){
            rank++;
        }
    }
}


void computeQRD(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
               Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & Q,
               Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & R,
               int & rank,
               Eigen::VectorXi & Pvec){
    //expect Mat=Jacobian.transpose()

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(Q.rows() == mRows);
    assert(Q.cols() == nCols);
    assert(R.rows() == nCols);
    assert(R.cols() == nCols);
    assert(Pvec.rows() == nCols);
    assert(Pvec.cols() == 1);

    Q.setZero();
    R.setZero();
    Pvec.setConstant(-1);

    if(Mat.isZero()){
        rank = 0;
        return;
    }

    unsigned int i = 0;
    unsigned int l;
    for(unsigned int k=0; k<=nCols-1; k++){
        if (changeRows){
            l=i;
        }
        else{
            l=k;
        }

        if (l >= mRows+1){
            break;
        }
        Q.col(l) = Mat.col(k);

        for(int j=0; j<=(int(l)-1); j++){
            R(j,l) = Q.col(j).transpose() * Mat.col(k);
            //R(j,l) = Q.col(j).transpose() * Q.col(l);
            Q.col(l) = Q.col(l) - Q.col(j) * (Q.col(l).transpose() * Q.col(j));
        }
        R(l,l) = Q.col(l).norm();
        if (R(l,l) > epsilon){
            Q.col(l) = Q.col(l) / R(l,l);
            Pvec(l) = k;
            i = i+1;
        }
    }
    rank = i;
}



void computeQRD2(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & Q,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & R,
                int & rank,
                Eigen::VectorXi & Pvec,
                bool const withoutPivoting){
    //expect Mat=Jacobian.transpose()

    //TODO: how to enforce that sorted rows in Mat are respected which is important for GHC?
    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(Q.rows() == mRows);
    assert(Q.cols() == nCols);
    assert(R.rows() == nCols);
    assert(R.cols() == nCols);
    assert(Pvec.rows() == nCols);
    assert(Pvec.cols() == 1);

    if(Mat.isZero()){
        Q.setZero();
        R.setZero();
        rank = 0;
        Pvec.setConstant(-1);
        return;
    }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Qtmp;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Rtmp;
    Qtmp=Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mRows,mRows);
    Rtmp=Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mRows,nCols);

    if (withoutPivoting){
//        qrDecomposition = Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(mRows, nCols);
        qrDecomposition.compute(Mat);
        //rank = nCols; //qrDecomposition is not rank-revealing
        Qtmp = qrDecomposition.householderQ();
        Rtmp = qrDecomposition.matrixQR();

        for(unsigned int k=0; k<nCols; k++){
            Pvec(k) = k;
        }
    }
    else{
//        qrDecompositionCol = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(mRows, nCols);
        qrDecompositionCol.compute(Mat);
        //rank = qrDecompositionCol.rank();
        Qtmp = qrDecompositionCol.householderQ().setLength(qrDecompositionCol.nonzeroPivots());
        //Rtmp = qrDecompositionCol.matrixQR(); //for full-rank matrices
        //Rtmp = qrDecompositionCol.matrixQR().topLeftCorner(rank, rank); //for rank-deficient matrices //TODO in our case wrong??
        Rtmp = qrDecompositionCol.matrixR().template triangularView<Eigen::Upper>(); //for full-rank matrices
        //Rtmp = qrDecompositionCol.matrixR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>(); //for rank-deficient matrices //TODO in our case wrong??

        //TODO what is the difference between these two matrices?
        //std::cout << "qrDecompositionCol.matrixQR()=" << qrDecompositionCol.matrixQR() << std::endl;
        //std::cout << "qrDecompositionCol.matrixR()=" << qrDecompositionCol.matrixR() << std::endl;

        assert(nCols == qrDecompositionCol.colsPermutation().indices().size());
        for(unsigned int k=0; k<nCols; k++){
            Pvec(k) = qrDecompositionCol.colsPermutation().indices()(k);
        }
    }
    assert(Qtmp.rows()==mRows);
    assert(Qtmp.cols()==mRows);
    assert(Rtmp.rows()==mRows);
    assert(Rtmp.cols()==nCols);

    Q = Qtmp.leftCols(nCols);
    R = Rtmp.topRows(nCols);

    assert(Q.rows() == mRows);
    assert(Q.cols() == nCols);
    assert(R.rows() == nCols);
    assert(R.cols() == nCols);

    rank = 0;
    for(unsigned int i=0; i<nCols; i++){
        if(fabs(R(i,i)) > 0){
            rank++;
        }
    }
}

//#############################################


bool checkOrthogonality(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat){
    return this->checkOrthogonality(Mat, 0.00001);
}


bool checkOrthogonality(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat, const double & threshold){
    //check if each combination of two task directions are orthogonal (Mat(k) * Mat(j)^T == 0)
    assert(threshold>0 && threshold<1);
    bool ok = true;
    double orthogonality;
    for(unsigned int a=0; a<Mat.cols(); a++){
        for(unsigned int b=a+1; b<Mat.cols(); b++){
            orthogonality = Mat.col(a).transpose() * Mat.col(b);
            if(std::isinf(orthogonality)){
                ok = false;
                std::cout << "orthogonality inf-issue for a=" << a << " and b=" << b << " => Mat(a)*Mat(b)^T = " << orthogonality << std::endl;
            }
            if(orthogonality!=orthogonality){
                ok = false;
                std::cout << "orthogonality nan-issue for a=" << a << " and b=" << b << " => Mat(a)*Mat(b)^T = " << orthogonality << std::endl;
            }
            if(orthogonality > threshold || orthogonality < -threshold){
                ok = false;
                std::cout << "orthogonality issue for a=" << a << " and b=" << b << " => Mat(a)*Mat(b)^T = " << orthogonality << std::endl;
            }
        }
    }
    return ok;
}

//#############################################


bool checkCHOLdecomposition(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & L,
            double & err1,
            double & err2,
            double & err3,
            double & err4,
            double & err5,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(mRows == nCols);
    assert(L.rows() == mRows);
    assert(L.cols() == mRows);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(mRows,mRows);

//    std::cout << "check CHOL decomposition" << std::endl;
//    std::cout << "Mat=\n" << Mat << std::endl;
//    std::cout << "L=\n" << L << std::endl;

    err1 = this->getAbsSumOfMatrix(Mat - L * L.transpose());//decomp of matrix
    err2 = this->getAbsSumOfMatrix(I - L.inverse() * L);
    err3 = this->getAbsSumOfMatrix(I - L * L.inverse());
    err4 = this->getAbsSumOfMatrix(I - L.transpose().inverse() * L.transpose());
    err5 = this->getAbsSumOfMatrix(I - L.transpose() * L.transpose().inverse());

    bool ok = true;
    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if(err2 > threshold || err2!=err2){
        ok = false;
    }
    if(err3 > threshold || err3!=err3){
        ok = false;
    }
    if(err4 > threshold || err4!=err4){
        ok = false;
    }
    if(err5 > threshold || err5!=err5){
        ok = false;
    }
    if (ok == false){
        std::cout << "check CHOL decomposition failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
        std::cout << "err2 = " << err2 << std::endl;
        std::cout << "err3 = " << err3 << std::endl;
        std::cout << "err4 = " << err4 << std::endl;
        std::cout << "err5 = " << err5 << std::endl;
    }
    return ok;
}


bool checkSVDdecomposition(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & U,
            Eigen::Matrix<double, Eigen::Dynamic, 1> const & S,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & V,
            int const & rank,
            double & err1,
            double & err2,
            double & err3,
            double & err4,
            double & err5,
            double & err6,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(U.rows() == mRows);
    assert(U.cols() == mRows);
    assert(S.rows() == mRows);
    assert(S.cols() == 1);
    assert(V.rows() == nCols);
    assert(V.cols() == mRows);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(mRows,mRows);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatPinv
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(nCols,mRows);
    MatPinv = Mat.transpose() * (Mat * Mat.transpose() + 0.0001 * I).inverse();

    Eigen::Matrix<double, Eigen::Dynamic, 1> Spinv = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(mRows);
    this->computeSpinv(S, Spinv);

//    std::cout << "check SVD decomposition" << std::endl;
//    std::cout << "rank = " << rank << std::endl;
//    std::cout << "Mat=\n" << Mat << std::endl;
//    std::cout << "MatPinv=\n" << MatPinv << std::endl;
//    std::cout << "U=\n" << U << std::endl;
//    std::cout << "S=\n" << S << std::endl;
//    std::cout << "V=\n" << V << std::endl;
//    std::cout << "Spinv=\n" << Spinv << std::endl;

    err1 = this->getAbsSumOfMatrix(Mat - U * S.asDiagonal() * V.transpose());//decomp of Pvecal matrix
    err2 = this->getAbsSumOfMatrix(MatPinv - V * Spinv.asDiagonal() * U.transpose());//decomp of pseudo inverse
    err3 = this->getAbsSumOfMatrix(I - U * U.transpose());
    err4 = this->getAbsSumOfMatrix(I - U.transpose() * U);
    err5 = this->getAbsSumOfMatrix(I - V.transpose() * V);
    err6 = this->getAbsSumOfMatrix(I - S.asDiagonal() * I * Spinv.asDiagonal())-rank;//TODO -rank ??

    bool ok = true;
    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if(err2 > threshold || err2!=err2){
        ok = false;
    }
    if(err3 > threshold || err3!=err3){
        ok = false;
    }
    if(err4 > threshold || err4!=err4){
        ok = false;
    }
    if(err5 > threshold || err5!=err5){
        ok = false;
    }
    if(err6 > threshold || err6!=err6){
        ok = false;
    }
    if (ok == false){
        std::cout << "check SVD decomposition failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
        std::cout << "err2 = " << err2 << std::endl;
        std::cout << "err3 = " << err3 << std::endl;
        std::cout << "err4 = " << err4 << std::endl;
        std::cout << "err5 = " << err5 << std::endl;
        std::cout << "err6 = " << err6 << std::endl;
    }
    return ok;
}


bool checkQRDdecomposition(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Q,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & R,
            Eigen::MatrixXi const & Pmat,
            int const & rank,
            double & err1,
            double & err2,
            double & err3,
            double & err4,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(Q.rows() == mRows);
    assert(Q.cols() == nCols);
    assert(R.rows() == nCols);
    assert(R.cols() == nCols);
    assert(Pmat.rows() == nCols);
    assert(Pmat.cols() == nCols);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(nCols,nCols);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatPinv
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(nCols,mRows);
    MatPinv = Mat.transpose() * (Mat * Mat.transpose() + 0.0001 * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(mRows,mRows)).inverse();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RTpinv_left
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(nCols,nCols);
    this->computeRpinv(R, rank, false, RTpinv_left);

//    std::cout << "check QRD decomposition" << std::endl;
//    std::cout << "rank = " << rank << std::endl;
//    std::cout << "Mat=\n" << Mat << std::endl;
//    std::cout << "MatPinv=\n" << MatPinv << std::endl;
//    std::cout << "Q=\n" << Q << std::endl;
//    std::cout << "R=\n" << R << std::endl;
//    std::cout << "RTpinv_left=\n" << RTpinv_left << std::endl;

    err1 = this->getAbsSumOfMatrix(Mat - Q * R * Pmat.transpose().cast<double>() );//decomp of Pvecal matrix
    err2 = this->getAbsSumOfMatrix(MatPinv - Pmat.cast<double>() * RTpinv_left * Q.transpose());//decomp of pseudo inverse
    err3 = this->getAbsSumOfMatrix(Q.transpose() * Q - I);//TODO add (-rank) for selfwritten QRD decomp
    err4 = this->getAbsSumOfMatrix(R * RTpinv_left - I)-rank;

    bool ok = true;
    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if(err2 > threshold || err2!=err2){
        ok = false;
    }
    if(err3 > threshold || err3!=err3){
        ok = false;
    }
    if(err4 > threshold || err4!=err4){
        ok = false;
    }
    if (ok == false){
        std::cout << "check QRD decomposition failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
        std::cout << "err2 = " << err2 << std::endl;
        std::cout << "err3 = " << err3 << std::endl;
        std::cout << "err4 = " << err4 << std::endl;
    }
    return ok;
}

//#############################################


bool checkPseudoinverse(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & pseudoinv,
            double & err1,
            double & err2,
            double & err3,
            double & err4,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(pseudoinv.rows() == nCols);
    assert(pseudoinv.cols() == mRows);

//    std::cout << "check pseudoinverse" << std::endl;

    //Four Penrose equations, see paper from F. Toutounian, A. Ataei
    //"A new method for computing Moore–Penrose inverse matrices", 2009
    //or wikipedia: https://en.wikipedia.org/wiki/Generalized_inverse#Types_of_generalized_inverses
    if(pseudoinv.isZero() && !Mat.isZero()){
        err1 = std::numeric_limits<double>::infinity();
        err2 = std::numeric_limits<double>::infinity();
        err3 = std::numeric_limits<double>::infinity();
        err4 = std::numeric_limits<double>::infinity();
    }
    else{
        err1 = this->getAbsSumOfMatrix(Mat * pseudoinv * Mat - Mat);
        err2 = this->getAbsSumOfMatrix(pseudoinv * Mat * pseudoinv - pseudoinv);
        err3 = this->getAbsSumOfMatrix( (Mat * pseudoinv).transpose() - Mat * pseudoinv);
        err4 = this->getAbsSumOfMatrix( (pseudoinv * Mat).transpose() - pseudoinv * Mat);
    }

    bool ok = true;
    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if(err2 > threshold || err2!=err2){
        ok = false;
    }
    if(err3 > threshold || err3!=err3){
        ok = false;
    }
    if(err4 > threshold || err4!=err4){
        ok = false;
    }
    if (ok == false){
        std::cout << "check pseudoinverse failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
        std::cout << "err2 = " << err2 << std::endl;
        std::cout << "err3 = " << err3 << std::endl;
        std::cout << "err4 = " << err4 << std::endl;
    }
    return ok;
}



bool checkPseudoinverseWeighted(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & pseudoinv,
            double & err1,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(W.rows() == nCols);
    assert(W.cols() == nCols);
    assert(pseudoinv.rows() == nCols);
    assert(pseudoinv.cols() == mRows);

//    std::cout << "check weighted pseudoinverse" << std::endl;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(mRows,mRows);

    //The first of all four Penrose equations has to be satisfied for a generalized inverse
    //https://en.wikipedia.org/wiki/Generalized_inverse#Types_of_generalized_inverses
    if(pseudoinv.isZero() && !Mat.isZero()){
        err1 = std::numeric_limits<double>::infinity();
    }
    else{
        err1 = this->getAbsSumOfMatrix(Mat * pseudoinv * Mat - Mat);
    }

    bool ok = true;
    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if (ok == false){
        std::cout << "check weighted pseudoinverse failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
    }
    return ok;
}


bool checkNullspace(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & nullspace,
            double & err1,
            double & err2,
            double & err3,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(nullspace.rows() == nCols);
    assert(nullspace.cols() == nCols);

//    std::cout << "check nullspace" << std::endl;

    if(nullspace.isZero()){
        err1 = std::numeric_limits<double>::infinity();
        err2 = std::numeric_limits<double>::infinity();
        err3 = std::numeric_limits<double>::infinity();
    }
    else{
        err1 = this->getAbsSumOfMatrix(nullspace * Mat.transpose());
        err2 = this->getAbsSumOfMatrix(nullspace * nullspace - nullspace);
        err3 = this->getAbsSumOfMatrix(nullspace.transpose() - nullspace);
        //TODO: for more nullspace properties see paper "Learning Null Space Projections" from Hsiu-Chin Lin (ICRA 2015)
    }

    bool ok = true;
    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if(err2 > threshold || err2!=err2){
        ok = false;
    }
    if(err3 > threshold || err3!=err3){
        ok = false;
    }
    if (ok == false){
        std::cout << "check nullspace failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
        std::cout << "err2 = " << err2 << std::endl;
        std::cout << "err3 = " << err3 << std::endl;
    }
    return ok;
}


bool checkNullspaceWeighted(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & nullspace,
            double & err1,
            const double & threshold) {

    unsigned int mRows = Mat.rows();
    unsigned int nCols = Mat.cols();
    assert(mRows > 0);
    assert(nCols > 0);
    assert(W.rows() == nCols);
    assert(W.cols() == nCols);
    assert(nullspace.rows() == nCols);
    assert(nullspace.cols() == nCols);

//    std::cout << "check weighted nullspace" << std::endl;


    if(nullspace.isZero()){
        err1 = std::numeric_limits<double>::infinity();
    }
    else{
        err1 = this->getAbsSumOfMatrix(nullspace * Mat.transpose());
        //TODO: for more nullspace properties see paper "Learning Null Space Projections" from Hsiu-Chin Lin (ICRA 2015)
    }

    bool ok = true;

    if(err1 > threshold || err1!=err1){
        ok = false;
    }
    if (ok == false){
        std::cout << "check weighted nullspace failed for threshold = " << threshold << std::endl;
        std::cout << "err1 = " << err1 << std::endl;
    }
    return ok;
}

//#############################################

//TODO delete this function?
void computeRtRinv(Eigen::Matrix<double, Eigen::Dynamic,
                   Eigen::Dynamic> const & R,
                   Eigen::VectorXi const & Pvec,
                   int const & rank,
                   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & RtRinv) {
    unsigned int dim = R.rows();
    assert(dim > 0);
    assert(R.rows() == R.cols());
    assert(RtRinv.rows() == dim);
    assert(RtRinv.cols() == dim);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtR
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(dim,dim);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtR_tmp
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rank,rank);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtRinv_tmp
            = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rank,rank);
    RtR = R.transpose() * R;
    RtRinv.setZero();

    if(RtR.isZero() || rank<1){
        return;
    }
    unsigned int counterRow, counterCol;

    counterRow = 0;
    for(unsigned int row=0; row<dim; row++){
        if (Pvec(row) >= 0){
            counterCol = 0;
            for(unsigned int col=0; col<dim; col++){
                if (Pvec(col) >= 0){
                    RtR_tmp(counterRow,counterCol) = RtR(Pvec(row),Pvec(col));
                    counterCol++;
                }
            }
            assert(int(counterCol)==rank);
            counterRow++;
        }
    }
    assert(int(counterRow)==rank);

    RtRinv_tmp = RtR_tmp.inverse();

    counterRow = 0;
    for(unsigned int row=0; row<dim; row++){
        if (Pvec(row) >= 0){
            counterCol = 0;
            for(unsigned int col=0; col<dim; col++){
                if (Pvec(col) >= 0){
                    RtRinv(Pvec(row),Pvec(col)) = RtRinv_tmp(counterRow,counterCol);
                    counterCol++;
                }
            }
            assert(int(counterCol)==rank);
            counterRow++;
        }
    }
    assert(int(counterRow)==rank);
//    std::cout << "RtR=\n" << RtR << std::endl;
//    std::cout << "RtR_tmp=\n" << RtR_tmp << std::endl;
//    std::cout << "RtRinv_tmp=\n" << RtRinv_tmp << std::endl;
//    std::cout << "RtRinv=\n" << RtRinv << std::endl;
}

void computeSpinv(Eigen::Matrix<double, Eigen::Dynamic, 1> const & S,
                  Eigen::Matrix<double, Eigen::Dynamic, 1> & Spinv) {
    unsigned int dim = S.rows();
    assert(dim > 0);
    assert(Spinv.rows() == dim);

    Spinv.setZero();
    for(unsigned int i=0; i<dim; i++){
        if(S(i)>0){
            Spinv(i) = 1.0 / S(i);
            if (Spinv(i) != Spinv(i) || std::isinf(Spinv(i))){
                Spinv(i) = 0.0;
            }
        }
    }
}

void computeRpinv(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & R,
                  int const & rank,
                  bool const doRightPinv,
                  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & Rpinv) {
    unsigned int dim = R.rows();
    assert(dim > 0);
    assert(R.rows() == R.cols());
    assert(Rpinv.rows() == dim);
    assert(Rpinv.cols() == dim);

    if(R.isZero() || rank==0){
        Rpinv.setZero();
        return;
    }

    if(doRightPinv){
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RRTinv
                = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(dim,dim);
        RRTinv.topLeftCorner(rank, rank) = (R.topLeftCorner(rank, rank) * R.topLeftCorner(rank, rank).transpose() ).inverse();
        Rpinv = R.transpose() * RRTinv;
    }
    else{
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RTRinv
                = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(dim,dim);
        RTRinv.topLeftCorner(rank, rank) = (R.topLeftCorner(rank, rank).transpose() * R.topLeftCorner(rank, rank) ).inverse();
        Rpinv = RTRinv * R.transpose();
    }

    //TODO there shouldn't be issues...
    for (unsigned int row = 0; row < dim; row++) {
        for (unsigned int col = 0; col < dim; col++) {
            if (std::isinf(Rpinv(row,col))) {
                Rpinv.setZero();
                std::cout << "inf-issue in computeRpinv function for rank=" << rank << " and R = \n" << R << std::endl;
                return;
            }
            if (Rpinv(row,col) != Rpinv(row,col)) {
                Rpinv.setZero();
                std::cout << "nan-issue in computeRpinv function for rank=" << rank << " and R = \n" << R << std::endl;
                return;
            }
        }
    }
    //TODO implement version2 from matlab
}

void computeSVDActivationMat(Eigen::Matrix<double, Eigen::Dynamic, 1> const & S,
                             Eigen::Matrix<double, Eigen::Dynamic, 1> & AsvdAuto) {
    unsigned int dim = S.rows();
    assert(dim > 0);
    assert(AsvdAuto.rows() == dim);

    AsvdAuto.setZero();
    for(unsigned int i=0; i<dim; i++){
        if(S(i) > borderA){
            AsvdAuto(i) = 1.0;
        }
        else{
            if(S(i) > borderB){
                AsvdAuto(i) = (S(i) - borderB) / (borderA - borderB); //linear interpolation between 0 and 1
                AsvdAuto(i) = 0.5 * (1.0-cos(AsvdAuto(i)*M_PI)); //sigmoidal interpolation between 0 and 1
            }
            else{
                AsvdAuto(i) = 0.0;
            }
        }
    }
}

void computeQRDActivationMat(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & R,
                             Eigen::Matrix<double, Eigen::Dynamic, 1> & AqrdAuto) {
    unsigned int dim = R.rows();
    assert(dim > 0);
    assert(R.cols() == dim);
    assert(AqrdAuto.rows() == dim);

    AqrdAuto.setZero();
    for(unsigned int i=0; i<dim; i++){
        if(fabs(R(i,i)) > borderA){
            AqrdAuto(i) = 1.0;
        }
        else{
            if(fabs(R(i,i)) > borderB){
                AqrdAuto(i) = (fabs(R(i,i)) - borderB) / (borderA - borderB); //linear interpolation between 0 and 1
                AqrdAuto(i) = 0.5 * (1.0-cos(AqrdAuto(i)*M_PI)); //sigmoidal interpolation between 0 and 1
            }
            else{
                AqrdAuto(i) = 0.0;
            }
        }
    }
}

void combineActivation(Eigen::Matrix<double, Eigen::Dynamic, 1> const & Aauto,
                       Eigen::Matrix<double, Eigen::Dynamic, 1> const & Auser,
                       Eigen::Matrix<double, Eigen::Dynamic, 1> & A){

    unsigned int dim = A.rows();
    assert(dim > 0);
    assert(Aauto.rows() == dim);
    assert(Auser.rows() == dim);

    if(useAauto && useAuser) {
//        A = (Aauto.asDiagonal() * Auser.asDiagonal()).diagonal(); //TODO: why does this line not compile??
        for(unsigned int i=0; i<dim; i++){
            A(i) = Aauto(i) * Auser(i);
        }
    }
    else if(useAauto && !useAuser) {
        A = Aauto;
    }
    else if(!useAauto && useAuser) {
        A = Auser;
    }
    else{
        A.setOnes();
    }
}

void permutationVec2Mat(Eigen::VectorXi const & Pvec, Eigen::MatrixXi & Pmat) {
    unsigned int dim = Pvec.rows();
    assert(dim > 0);
    assert(Pmat.rows() == dim);
    assert(Pmat.cols() == dim);

    Pmat.setZero();
    for(unsigned int row=0; row<dim; row++){
        if (Pvec(row)>=0){
            Pmat(Pvec(row),row)=1;
        }
    }
}

void permutationMat2Vec(Eigen::MatrixXi const & Pmat, Eigen::VectorXi & Pvec) {
    int dim = Pvec.rows();
    assert(dim > 0);
    assert(Pmat.rows() == dim);
    assert(Pmat.cols() == dim);

    Pvec.setConstant(-1);
    for(int col=0; col<dim; col++){
        for(int row=0; row<dim; row++){
            if (Pmat(row,col)==1){
                Pvec(col) = row;
            }
        }
    }
}


double getAbsSumOfMatrix(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat) {
    if (Mat.rows() == 0 || Mat.cols() == 0){
        throw std::runtime_error("getAbsSumOfMatrix => mat has zero rows or cols");
    }
    assert(Mat.rows() > 0 && Mat.cols() > 0);

    double sum = 0.0;
    for(unsigned int r=0; r<Mat.rows(); r++){
        for(unsigned int c=r; c<Mat.cols(); c++){
            sum = sum + fabs(Mat(r,c));
        }
    }
    return sum;
}

void test(unsigned int rows, unsigned int cols) {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Mat;
    Mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Random(rows,cols);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W;
    W = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Random(cols,cols);
    W = W*W.transpose();

    this->test(Mat, W, 0.0);
}


void test(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & Mat,
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> const & W,
          double const reg) {
    unsigned int rows = Mat.rows();
    unsigned int cols = Mat.cols();
    assert(rows > 0);
    assert(cols > 0);
    assert(W.rows() == cols);
    assert(W.cols() == cols);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L;
    L = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pseudoinv;
    pseudoinv = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> nullspace;
    nullspace = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,cols);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V, U;
    V = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);

    Eigen::Matrix<double, Eigen::Dynamic, 1> S;
    S = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q, R;
    Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(cols,rows);
    R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows,rows);

    Eigen::VectorXi Pvec;
    Pvec = Eigen::VectorXi::Zero(rows);

    Eigen::MatrixXi Pmat;
    Pmat = Eigen::MatrixXi::Zero(rows,rows);

    Eigen::Matrix<double, Eigen::Dynamic, 1> AsvdAuto, AqrdAuto, AsvdUser, AqrdUser;
    AsvdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);
    AqrdAuto = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(rows);
    AsvdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);
    AqrdUser = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(rows);

    int rankSVD = -1;
    int rankQRD = -1;

    double err1, err2, err3, err4, err5, err6;

    // ####################################################################

    std::cout << "### test CHOL decomposition" << std::endl;
    this->computeCHOL(W, L);
    this->checkCHOLdecomposition(W, L, err1, err2, err3, err4, err5, 0.01);

    std::cout << "### test SVD decomposition" << std::endl;
    this->computeSVD(Mat, U, S, V, rankSVD);
    this->computeSVDActivationMat(S, AsvdAuto);
    this->checkSVDdecomposition(Mat, U, S, V, rankSVD, err1, err2, err3, err4, err5, err6, 0.001);
    this->checkOrthogonality(V, 0.01);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;
//    std::cout << "err5 = " << err5 << std::endl;
//    std::cout << "err6 = " << err6 << std::endl;
//    std::cout << "Mat \n" << Mat << std::endl;
//    std::cout << "U \n" << U << std::endl;
//    std::cout << "S \n" << S << std::endl;
//    std::cout << "V \n" << V << std::endl;
//    std::cout << "AsvdAuto \n" << AsvdAuto << std::endl;
//    std::cout << "rankSVD " << rankSVD << std::endl;


//    std::cout << "### test QRD decomposition" << std::endl;
//    this->computeQRD(Mat.transpose(), Q, R, rankQRD, Pvec);
//    this->computeQRDActivationMat(R, AqrdAuto);
//    this->permutationVec2Mat(Pvec, Pmat);
//    this->checkQRDdecomposition(Mat.transpose(), Q, R, Pmat, rankQRD, err1, err2, err3, err4, 0.001);
//    this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;
//    std::cout << "Mat \n" << Mat << std::endl;
//    std::cout << "Q \n" << Q << std::endl;
//    std::cout << "R \n" << R << std::endl;
//    std::cout << "Pvec \n" << Pvec << std::endl;
//    std::cout << "Pmat \n" << Pmat << std::endl;
//    std::cout << "AqrdAuto \n" << AqrdAuto << std::endl;
//    std::cout << "rankQRD " << rankQRD << std::endl;


//    std::cout << "### test QRD2 decomposition without Pivoting" << std::endl;
//    this->computeQRD2(Mat.transpose(), Q, R, rankQRD, Pvec, true);
//    this->computeQRDActivationMat(R, AqrdAuto);
//    this->permutationVec2Mat(Pvec, Pmat);
//    this->checkQRDdecomposition(Mat.transpose(), Q, R, Pmat, rankQRD, err1, err2, err3, err4, 0.001);
//    this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;
//    std::cout << "Mat \n" << Mat << std::endl;
//    std::cout << "Q \n" << Q << std::endl;
//    std::cout << "R \n" << R << std::endl;
//    std::cout << "Pvec \n" << Pvec << std::endl;
//    std::cout << "Pmat \n" << Pmat << std::endl;
//    std::cout << "AqrdAuto \n" << AqrdAuto << std::endl;
//    std::cout << "rankQRD " << rankQRD << std::endl;


    std::cout << "### test QRD2 decomposition with Pivoting" << std::endl;
    this->computeQRD2(Mat.transpose(), Q, R, rankQRD, Pvec, false);
    this->computeQRDActivationMat(R, AqrdAuto);
    this->permutationVec2Mat(Pvec, Pmat);
    this->checkQRDdecomposition(Mat.transpose(), Q, R, Pmat, rankQRD, err1, err2, err3, err4, 0.001);
    this->checkOrthogonality(Q, 0.01); //if issues appear here, eventually increase epsilon...
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;
//    std::cout << "Mat \n" << Mat << std::endl;
//    std::cout << "Q \n" << Q << std::endl;
//    std::cout << "R \n" << R << std::endl;
//    std::cout << "Pvec \n" << Pvec << std::endl;
//    std::cout << "Pmat \n" << Pmat << std::endl;
//    std::cout << "AqrdAuto \n" << AqrdAuto << std::endl;
//    std::cout << "rankQRD " << rankQRD << std::endl;


    // ####################################################################

    std::cout << "### test DLS pseudoinverse" << std::endl;
    this->computeDLSpseudoinv(Mat, reg, pseudoinv);
    this->checkPseudoinverse(Mat, pseudoinv, err1, err2, err3, err4, 0.1);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;

    std::cout << "### test SVD pseudoinverse" << std::endl;
    this->computeSVDpseudoinv(Mat, AsvdUser, pseudoinv, rankSVD, AsvdAuto);
    this->checkPseudoinverse(Mat, pseudoinv, err1, err2, err3, err4, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;

    std::cout << "### test QRD pseudoinverse" << std::endl;
    this->computeQRDpseudoinv(Mat, AqrdUser, pseudoinv, rankQRD, AqrdAuto);
    this->checkPseudoinverse(Mat, pseudoinv, err1, err2, err3, err4, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;
//    std::cout << "err4 = " << err4 << std::endl;

    std::cout << "### test DLS pseudoinverse weighted" << std::endl;
    this->computeDLSpseudoinvWeighted(Mat, W, reg, pseudoinv);
    this->checkPseudoinverseWeighted(Mat, W, pseudoinv, err1, 0.1);
//    std::cout << "err1 = " << err1 << std::endl;

    std::cout << "### test SVD pseudoinverse weighted" << std::endl;
    this->computeSVDpseudoinvWeighted(Mat, W, AsvdUser, pseudoinv, rankSVD, AsvdAuto);
    this->checkPseudoinverseWeighted(Mat, W, pseudoinv, err1, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;

    std::cout << "### test QRD pseudoinverse weighted" << std::endl;
    this->computeQRDpseudoinvWeighted(Mat, W, AqrdUser, pseudoinv, rankQRD, AqrdAuto);
    this->checkPseudoinverseWeighted(Mat, W, pseudoinv, err1, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;

    // ####################################################################

    std::cout << "### test DLS nullspace" << std::endl;
    this->computeDLSnullspace(Mat, reg, nullspace);
    this->checkNullspace(Mat, nullspace, err1, err2, err3, 0.1);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;

    std::cout << "### test SVD nullspace" << std::endl;
    this->computeSVDnullspace(Mat, AsvdUser, nullspace, rankSVD, AsvdAuto);
    this->checkNullspace(Mat, nullspace, err1, err2, err3, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;

    std::cout << "### test QRD nullspace" << std::endl;
    this->computeQRDnullspace(Mat, AqrdUser, nullspace, rankQRD, AqrdAuto);
    this->checkNullspace(Mat, nullspace, err1, err2, err3, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;
//    std::cout << "err2 = " << err2 << std::endl;
//    std::cout << "err3 = " << err3 << std::endl;

    std::cout << "### test DLS nullspace weighted" << std::endl;
    this->computeDLSnullspaceWeighted(Mat, W, reg, nullspace);
    this->checkNullspaceWeighted(Mat, W, nullspace, err1, 0.01);
//    std::cout << "err1 = " << err1 << std::endl;

    std::cout << "### test SVD nullspace weighted" << std::endl;
    this->computeSVDnullspaceWeighted(Mat, W, AsvdUser, nullspace, rankSVD, AsvdAuto);
    this->checkNullspaceWeighted(Mat, W, nullspace, err1, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;

    std::cout << "### test QRD nullspace weighted" << std::endl;
    this->computeQRDnullspaceWeighted(Mat, W, AqrdUser, nullspace, rankQRD, AqrdAuto);
    this->checkNullspaceWeighted(Mat, W, nullspace, err1, 0.0001);
//    std::cout << "err1 = " << err1 << std::endl;

    // ####################################################################
}

double epsilon;
double borderA, borderB;
bool changeRows;
bool useAauto, useAuser;

protected:
    Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > qrDecomposition;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > qrDecompositionCol;
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > svdJacobi;
    Eigen::BDCSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > svdBDC;
    Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > chol;
    Eigen::LDLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > cholPivot;
};

}

}