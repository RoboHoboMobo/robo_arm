#include "../include/kinematics.h"

// Constructors
Kinematics::Kinematics(double val)
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = val;
}

Kinematics::Kinematics(double m00, double m01, double m02, double m03,
                       double m10, double m11, double m12, double m13,
                       double m20, double m21, double m22, double m23,
                       double m30, double m31, double m32, double m33)
{
  m[0][0] = m00;  m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
  m[1][0] = m10;  m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
  m[2][0] = m20;  m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
  m[3][0] = m30;  m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
}

Kinematics::Kinematics(double arr[16])
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = arr[i*4 + j];
}

Kinematics::Kinematics(double arr[4][4])
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = arr[i][j];
}

Kinematics::Kinematics(std::vector<double> vect)
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = vect[i*4 + j];
}

// Operators overload
Kinematics Kinematics::operator -() const
{
  Kinematics matr;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      matr.m[i][j] = -m[i][j];

  return matr;

}

Kinematics Kinematics::operator -(Kinematics const &right) const
{
  Kinematics result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = m[i][j] - right.m[i][j];

  return result;
}

void Kinematics::operator -=(Kinematics const &right)
{
  *this = *this - right;
}

Kinematics Kinematics::operator +() const
{
  return *this;
}

Kinematics Kinematics::operator +(Kinematics const &right) const
{
  Kinematics result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = m[i][j] + right.m[i][j];

  return result;
}

void Kinematics::operator +=(Kinematics const &right)
{
  *this = *this + right;
}

Kinematics Kinematics::operator*(double right) const
{
  Kinematics result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = m[i][j] * right;

  return result;
}

Kinematics Kinematics::operator*(Kinematics const &right) const
{
  Kinematics result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      for(int k=0; k<4; k++)
        result.m[i][j] += m[i][k] * right.m[k][j];

  return result;

}

void Kinematics::operator *=(Kinematics const &right)
{
  *this = *this * right;
}

void Kinematics::operator*=(double right)
{
  *this = *this * right;
}

Kinematics operator*(double left, Kinematics const &right)
{
  Kinematics result;
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = left * right.m[i][j];

  return result;
}

// Methods
double Kinematics::getValue(int row, int col)
{
  return m[row][col];
}

void Kinematics::getTMatrix(double arr[4][4])
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      arr[i][j] = m[i][j];

}

Kinematics Kinematics::DHToM(double alph_i_1, double a_i_1, double d_i, double thet_i)
{
  Kinematics Ti_i_1(              cos(thet_i),              -sin(thet_i),            0.0,              a_i_1,
                    sin(thet_i)*cos(alph_i_1), cos(thet_i)*cos(alph_i_1), -sin(alph_i_1), -sin(alph_i_1)*d_i,
                    sin(thet_i)*sin(alph_i_1), cos(thet_i)*sin(alph_i_1),  cos(alph_i_1),  cos(alph_i_1)*d_i,
                                          0.0,                       0.0,            0.0,                1.0);

  return Ti_i_1;

}

Kinematics Kinematics::getFK(double th1, double th2, double th3, double th4, double th5, double th6)
{
  Kinematics result;

  result = Kinematics::DHToM(0.0, 0.0, L1, th1) * Kinematics::DHToM(-M_PI/2, 0.0, 0.0, th2-M_PI/2) *
           Kinematics::DHToM(0.0, L2, 0.0, th3) * Kinematics::DHToM(0.0, L3, 0.0, th4) *
           Kinematics::DHToM(0.0, L4, 0.0, th5) * Kinematics::DHToM(0.0, L5, 0.0, 0.0) *
           Kinematics::DHToM(0.0, F0*cos(th6), 0.0, 0.0) * Kinematics::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

Kinematics Kinematics::getFK(double j_angles[6])
{
  Kinematics result;
  result = Kinematics::DHToM(0.0, 0.0, L1, j_angles[0]) * Kinematics::DHToM(-M_PI/2, 0.0, 0.0, j_angles[1]-M_PI/2) *
           Kinematics::DHToM(0.0, L2, 0.0, j_angles[2]) * Kinematics::DHToM(0.0, L3, 0.0, j_angles[3]) *
           Kinematics::DHToM(0.0, L4, 0.0, j_angles[4]) * Kinematics::DHToM(0.0, L5, 0.0, 0.0) *
           Kinematics::DHToM(0.0, F0*cos(j_angles[5]), 0.0, 0.0) * Kinematics::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

Kinematics Kinematics::getFK(std::vector<double> j_angles)
{
  Kinematics result;
  result = Kinematics::DHToM(0.0, 0.0, L1, j_angles[0]) * Kinematics::DHToM(-M_PI/2, 0.0, 0.0, j_angles[1]-M_PI/2) *
           Kinematics::DHToM(0.0, L2, 0.0, j_angles[2]) * Kinematics::DHToM(0.0, L3, 0.0, j_angles[3]) *
           Kinematics::DHToM(0.0, L4, 0.0, j_angles[4]) * Kinematics::DHToM(0.0, L5, 0.0, 0.0) *
           Kinematics::DHToM(0.0, F0*cos(j_angles[5]), 0.0, 0.0) * Kinematics::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

void Kinematics::getRPY(double &r, double &p, double &y)
{
  if(m[0][2] < 1)
  {
    if(m[0][2] > -1)
    {
      r = atan2(-m[1][2], m[2][2]); // Roll
      p = asin(m[0][2]);            // Pitch
      y = atan2(-m[0][1], m[0][0]);
    }
    else
    {
      r = -atan2(m[1][0], m[1][1]);
      p = -M_PI/2;
      y = 0.0;
    }
  }
  else
  {
    r = atan2(m[1][0], m[1][1]);
    p = M_PI/2;
    y = 0.0;

  }

}
