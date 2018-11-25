#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main() {

    MatrixXd m = MatrixXd::Constant(3, 3, 2.0);
    MatrixXd c = MatrixXd::Constant(3, 1, 1.2);
    MatrixXd a = m*c;
    MatrixXd b = MatrixXd::Constant(1, 3, 5.0);
    VectorXd d(7);
    d << 1, 2, 3, 4, 5, 6, 7;
    //cout << a << endl;
    cout << (b*a) << endl;
    cout << d << endl;

    Matrix2d e;
    e << 1, 1, 1, 1;
    cout << e.determinant() << endl;
    cout << e.reverse() << endl;

    return 0;
}