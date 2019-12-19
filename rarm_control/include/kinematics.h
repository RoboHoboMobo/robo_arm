#include <iostream>
#include <cmath>
#include <vector>

class Kinematics
{
private:
  double m[4][4];

public:
  Kinematics();
  Kinematics(double m00, double m01, double m02, double m03,
             double m10, double m11, double m12, double m13,
             double m20, double m21, double m22, double m23,
             double m30, double m31, double m32, double m33);

  Kinematics(double arr[16]);
  Kinematics(double arr[4][4]);
  Kinematics(std::vector<double> vect);

  double getValue(int row, int col);
  void getTMatrix(double arr[4][4]);

  Kinematics operator-() const;
  Kinematics operator-(Kinematics const &right) const;
  void operator-=(Kinematics const &right);

  Kinematics operator+() const;
  Kinematics operator+(Kinematics const &right) const;
  void operator+=(Kinematics const &right);

  Kinematics operator*(double right) const;
  Kinematics operator*(Kinematics const &right) const;
  void operator*=(Kinematics const &right);
  void operator*=(double right);

  friend Kinematics operator*(double left, Kinematics const &right);


};


Kinematics operator*(double left, Kinematics const &right);
