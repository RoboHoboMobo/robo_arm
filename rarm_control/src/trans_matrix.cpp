#include "../include/trans_matrix.h"

// Constructors
TransMatrix::TransMatrix(double val)
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = val;
}

TransMatrix::TransMatrix(double m00, double m01, double m02, double m03,
                         double m10, double m11, double m12, double m13,
                         double m20, double m21, double m22, double m23,
                         double m30, double m31, double m32, double m33)
{
  m[0][0] = m00;  m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
  m[1][0] = m10;  m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
  m[2][0] = m20;  m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
  m[3][0] = m30;  m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
}

TransMatrix::TransMatrix(double arr[16])
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = arr[i*4 + j];
}

TransMatrix::TransMatrix(double arr[4][4])
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = arr[i][j];
}

TransMatrix::TransMatrix(std::vector<double> vect)
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      m[i][j] = vect[i*4 + j];
}

// Operators overload
TransMatrix TransMatrix::operator -() const
{
  TransMatrix result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = -m[i][j];

  return result;

}

TransMatrix TransMatrix::operator -(TransMatrix const &right) const
{
  TransMatrix result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = m[i][j] - right.m[i][j];

  return result;
}

void TransMatrix::operator -=(TransMatrix const &right)
{
  *this = *this - right;
}

TransMatrix TransMatrix::operator +() const
{
  return *this;
}

TransMatrix TransMatrix::operator +(TransMatrix const &right) const
{
  TransMatrix result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = m[i][j] + right.m[i][j];

  return result;
}

void TransMatrix::operator +=(TransMatrix const &right)
{
  *this = *this + right;
}

TransMatrix TransMatrix::operator*(double right) const
{
  TransMatrix result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = m[i][j] * right;

  return result;
}

TransMatrix TransMatrix::operator*(TransMatrix const &right) const
{
  TransMatrix result;

  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      for(int k=0; k<4; k++)
        result.m[i][j] += m[i][k] * right.m[k][j];

  return result;

}

void TransMatrix::operator *=(TransMatrix const &right)
{
  *this = *this * right;
}

void TransMatrix::operator*=(double right)
{
  *this = *this * right;
}

TransMatrix operator*(double left, TransMatrix const &right)
{
  TransMatrix result;
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      result.m[i][j] = left * right.m[i][j];

  return result;
}

// Methods
double TransMatrix::getValue(int row, int col)
{
  return m[row][col];
}

void TransMatrix::getTMatrix(double arr[4][4])
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
      arr[i][j] = m[i][j];

}

void TransMatrix::getRPY(double &r, double &p, double &y)
{
  if(m[0][2] < 1)
  {
    if(m[0][2] > -1)
    {
      r = atan2(-m[1][2], m[2][2]); // Roll
      p = asin(m[0][2]);            // Pitch
      y = atan2(-m[0][1], m[0][0]); // Yaw
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

TransMatrix TransMatrix::DHToM(double alph_i_1, double a_i_1, double d_i, double thet_i)
{
  TransMatrix Ti_i_1(              cos(thet_i),              -sin(thet_i),            0.0,              a_i_1,
                     sin(thet_i)*cos(alph_i_1), cos(thet_i)*cos(alph_i_1), -sin(alph_i_1), -sin(alph_i_1)*d_i,
                     sin(thet_i)*sin(alph_i_1), cos(thet_i)*sin(alph_i_1),  cos(alph_i_1),  cos(alph_i_1)*d_i,
                                           0.0,                       0.0,            0.0,                1.0);

  return Ti_i_1;

}

TransMatrix TransMatrix::RPYtoM(double r, double p, double y)
{
  TransMatrix result;

  TransMatrix mr(1.0,    0.0,     0.0, 0.0,
                 0.0, cos(r), -sin(r), 0.0,
                 0.0, sin(r),  cos(r), 0.0,
                 0.0,    0.0,     0.0, 1.0);

  TransMatrix mp( cos(p), 0.0, sin(p), 0.0,
                     0.0, 1.0,    0.0, 0.0,
                 -sin(p), 0.0, cos(p), 0.0,
                     0.0, 0.0,    0.0, 1.0);

  TransMatrix my(cos(y), -sin(y), 0.0, 0.0,
                 sin(y),  cos(y), 0.0, 0.0,
                    0.0,     0.0, 1.0, 0.0,
                    0.0,     0.0, 0.0, 1.0);

  result = mr * mp * my;

  return result;

}
