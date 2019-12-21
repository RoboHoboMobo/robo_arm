#include <iostream>
#include <cmath>
#include <vector>

class TransMatrix
{
private:
  double m[4][4];

public:
// Constructors
  TransMatrix(double val = 0.0);
  TransMatrix(double m00, double m01, double m02, double m03,
              double m10, double m11, double m12, double m13,
              double m20, double m21, double m22, double m23,
              double m30, double m31, double m32, double m33);

  TransMatrix(double arr[16]);
  TransMatrix(double arr[4][4]);
  TransMatrix(std::vector<double> vect);

// Operators overload
  TransMatrix operator-() const;
  TransMatrix operator-(TransMatrix const &right) const;
  void operator-=(TransMatrix const &right);

  TransMatrix operator+() const;
  TransMatrix operator+(TransMatrix const &right) const;
  void operator+=(TransMatrix const &right);

  TransMatrix operator*(double right) const;
  TransMatrix operator*(TransMatrix const &right) const;
  void operator*=(TransMatrix const &right);
  void operator*=(double right);

  friend TransMatrix operator*(double left, TransMatrix const &right);

// Methods
  double getValue(int row, int col);
  void getTMatrix(double arr[4][4]);
  void getRPY(double &r, double &p, double &y);
  static TransMatrix DHToM(double alph_i_1, double a_i_1, double d_i, double thet_i); // DH-table row to Transformation matrix conversion
};

TransMatrix operator*(double left, TransMatrix const &right);
