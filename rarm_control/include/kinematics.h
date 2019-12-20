#include <iostream>
#include <cmath>
#include <vector>

#define L1 0.041  // Links lengts
#define L2 0.090
#define L3 0.090
#define L4 0.090
#define L5 0.049
#define F0 0.0459 // Gripper fingers' lengths
#define F1 0.060


class Kinematics
{
private:
  double m[4][4];

public:
// Constructors
  Kinematics(double val = 0.0);
  Kinematics(double m00, double m01, double m02, double m03,
             double m10, double m11, double m12, double m13,
             double m20, double m21, double m22, double m23,
             double m30, double m31, double m32, double m33);

  Kinematics(double arr[16]);
  Kinematics(double arr[4][4]);
  Kinematics(std::vector<double> vect);

// Operators overload
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

// Methods
  double getValue(int row, int col);
  void getTMatrix(double arr[4][4]);
  static Kinematics DHToM(double alph_i_1, double a_i_1, double d_i, double thet_i); // DH-table row to Transformation matrix conversion
  static Kinematics getFK(double th1, double th2, double th3, double th4, double th5, double th6);
  static Kinematics getFK(double j_angles[6]);
  static Kinematics getFK(std::vector<double> j_angles);
  void getRPY(double &r, double &p, double &y);


};


Kinematics operator*(double left, Kinematics const &right);
