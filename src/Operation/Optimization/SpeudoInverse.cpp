#include "SpeudoInverse.h"
#include <armadillo>

//Constructor / Destructor
SpeudoInverse::SpeudoInverse(){}
SpeudoInverse::~SpeudoInverse(){}

MatrixXf SpeudoInverse::SpeudoInverse_orthoDecomp(MatrixXf J){
  //---------------------------

  MatrixXf Ji = J.completeOrthogonalDecomposition().pseudoInverse();

  if(Ji.determinant() == 0){
    cout<<"Problem inversion matrix !"<<endl;
  }

  //---------------------------
  return Ji;
}
MatrixXf SpeudoInverse::SpeudoInverse_QRDecomp(MatrixXf J){
  MatrixXf A, C, Ji;
  //---------------------------

  A = (J.transpose() * J);
  Eigen::ColPivHouseholderQR<MatrixXf> qr(A);
  C = qr.inverse();
  Ji = C*J.transpose();

  if(!qr.isInvertible()){
    cout<<"Problem inversion matrix !"<<endl;
  }

  //---------------------------
  return Ji;
}
MatrixXf SpeudoInverse::SpeudoInverse_LUDecomp(MatrixXf J){
  MatrixXf A, C, Ji;
  //---------------------------

  A = (J.transpose() * J);
  Eigen::FullPivLU<MatrixXf> lu(A);
  C = lu.inverse();
  Ji = C*J.transpose();

  if(!lu.isInvertible()){
    cout<<"Problem inversion matrix !"<<endl;
  }

  //---------------------------
  return Ji;
}
MatrixXf SpeudoInverse::SpeudoInverse_armadillo(MatrixXf J){
  arma::mat A(J.rows(), J.cols());
  //---------------------------

  for(int i=0; i<J.rows(); i++){
    for(int j=0; j<J.cols(); j++){
      A(i,j) = J(i,j);
    }
  }

  arma::mat Ji = arma::pinv(A);

  MatrixXf Jinv(Ji.n_rows, Ji.n_cols);
  for(int i=0; i<Ji.n_rows; i++){
    for(int j=0; j<Ji.n_cols; j++){
      Jinv(i,j) = Ji(i,j);
    }
  }

  //---------------------------
  return Jinv;
}
