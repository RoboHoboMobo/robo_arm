#include "../include/kinematics.h"

Kinematics::Kinematics()
{
  this->th1 = 0.0; this->th2 = 0.0; this->th3 = 0.0;
  this->th4 = 0.0; this->th5 = 0.0; this->th6 = 0.0;

  TransMatrix m(1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0);

  this->ee_pose = m;

  this->theta = 0.0;

  k = 0.9;
}

Kinematics::Kinematics(double th1, double th2, double th3,
                       double th4, double th5, double th6)
{
  this->th1 = th1; this->th2 = th2; this->th3 = th3;
  this->th4 = th4; this->th5 = th5; this->th6 = th6;
  this->theta = th1 + th2 + th3 + th4 + th5 - M_PI/2;

  this->ee_pose = Kinematics::solveFK(th1, th2, th3, th4, th5, th6);

}

Kinematics::Kinematics(double x, double y, double z, double theta, double grip_angle)
{
  this->theta = theta;
  double a[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  Kinematics::solveIK(x, y, z, theta, grip_angle, a);

  th1 = a[0]; th2 = a[1]; th3 = a[2];
  th4 = a[3]; th5 = a[4]; th6 = grip_angle;

  this->ee_pose = Kinematics::solveFK(th1, th2, th3, th4, th5, th6);

}

Kinematics::Kinematics(TransMatrix tm)
{
  ee_pose = tm;

  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  tm.getRPY(r, p, y);
  theta = p - M_PI/2;

  double a[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  Kinematics::solveIK(tm.getValue(0,3), tm.getValue(1,3), tm.getValue(2,3), p - M_PI/2, 0.0, a);

  th1 = a[0]; th2 = a[1]; th3 = a[2];
  th4 = a[3]; th5 = a[4]; th6 = 0.0;

}

TransMatrix Kinematics::solveFK()
{
  TransMatrix result;

  result = TransMatrix::DHToM(0.0, 0.0, L1, th1)          * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, th2-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, th3)          * TransMatrix::DHToM(0.0, L3, 0.0, th4) *
           TransMatrix::DHToM(0.0, L4, 0.0, th5 + M_PI/2) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(th6), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

TransMatrix Kinematics::solveFK(double th1, double th2, double th3, double th4, double th5, double th6)
{
  TransMatrix result;

  result = TransMatrix::DHToM(0.0, 0.0, L1, th1)          * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, th2-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, th3)          * TransMatrix::DHToM(0.0, L3, 0.0, th4) *
           TransMatrix::DHToM(0.0, L4, 0.0, th5 + M_PI/2) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(th6), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

TransMatrix Kinematics::solveFK(double j_angles[6])
{
  TransMatrix result;
  result = TransMatrix::DHToM(0.0, 0.0, L1, j_angles[0])          * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, j_angles[1]-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, j_angles[2])          * TransMatrix::DHToM(0.0, L3, 0.0, j_angles[3]) *
           TransMatrix::DHToM(0.0, L4, 0.0, j_angles[4] + M_PI/2) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(j_angles[5]), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

TransMatrix Kinematics::solveFK(std::vector<double> j_angles)
{
  TransMatrix result;
  result = TransMatrix::DHToM(0.0, 0.0, L1, j_angles[0])          * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, j_angles[1]-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, j_angles[2])          * TransMatrix::DHToM(0.0, L3, 0.0, j_angles[3]) *
           TransMatrix::DHToM(0.0, L4, 0.0, j_angles[4] + M_PI/2) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(j_angles[5]), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

void Kinematics::setK(double k)
{
  if(k > 1.0)
  {
    std::cout<<"Error: wrong vaulue of k-parameter!"<<std::endl;
    return;
  }
  if(k < 0.0)
  {
    std::cout<<"Error: wrong vaulue of k-parameter!"<<std::endl;
    return;
  }

  this->k = k;

}

void Kinematics::solveIK(double result[5])
{
  double x = ee_pose.getValue(0,3);
  double y = ee_pose.getValue(1,3);
  double z = ee_pose.getValue(2,3);
  double alpha1 = atan2(y, x);                                // 1st joint's angle

  double x5 = x - (L5 + F0*cos(th6) + F1) * cos(theta)*cos(alpha1);
  double z5 = z + (L5 + F0*cos(th6) + F1) * sin(theta);

  double x4 = x5 - L4 * sin((1-k)*(M_PI/2 + theta))*cos(alpha1); // some conditions for IK simplification
  double z4 = z5 - L4 * cos((1-k)*(M_PI/2 + theta));

  double r4 = sqrt(x4*x4 + z4*z4);
  double beta = atan2(z4, x4);

  double r3 = sqrt(L1*L1 + r4*r4 - 2*L1*r4*sin(beta));
  double delta = atan2(x4, z4 - L1);

  double s_d_a2 = 1/(2*r3*L2)*sqrt(((z4-L1)*(z4-L1) - (L3-L2)*(L3-L2)) * ((L2+L3)*(L2+L3) - (z4-L1)*(z4-L1)));
  double c_d_a2 = (L2*L2 + r3*r3 - L3*L3)/(2*L2*r3);

  double alpha2 = delta - atan2(s_d_a2, c_d_a2);               // 2nd joint's angle

  double s_a3 = r3*sin(delta - alpha2)/L3;
  double c_a3 = ((z4-L1)*(z4-L1) + x4*x4 -L3*L3 -L2*L2)/(2*L2*L3);
  double alpha3 = atan2(s_a3, c_a3);                          // 3rd joint's angle

  double alpha4 = (1-k)*(M_PI/2 + theta) - (alpha2 + alpha3); // 4th joint's angle
  double alpha5 = k*(M_PI/2 + theta);                         // 5th joint's angle

  result[0] = alpha1;
  result[1] = alpha2;
  result[2] = alpha3;
  result[3] = alpha4;
  result[4] = alpha5;

}

bool Kinematics::solveIK(std::vector<double> &angles)
{
    double x = ee_pose.getValue(0,3);
    double y = ee_pose.getValue(1,3);
    double z = ee_pose.getValue(2,3);
    double alpha1 = atan2(y, x);                                // 1st joint's angle

    double x5 = x - (L5 + F0*cos(th6) + F1) * cos(theta)*cos(alpha1);
    double z5 = z + (L5 + F0*cos(th6) + F1) * sin(theta);

    double x4 = x5 - L4 * sin((1-k)*(M_PI/2 + theta))*cos(alpha1); // some conditions for IK simplification
    double z4 = z5 - L4 * cos((1-k)*(M_PI/2 + theta));

    double r4 = sqrt(x4*x4 + z4*z4);
    double beta = atan2(z4, x4);

    double r3 = sqrt(L1*L1 + r4*r4 - 2*L1*r4*sin(beta));
    double delta = atan2(x4, z4 - L1);

    double s_d_a2 = 1/(2*r3*L2)*sqrt(((z4-L1)*(z4-L1) - (L3-L2)*(L3-L2)) * ((L2+L3)*(L2+L3) - (z4-L1)*(z4-L1)));
    double c_d_a2 = (L2*L2 + r3*r3 - L3*L3)/(2*L2*r3);

    double alpha2 = delta - atan2(s_d_a2, c_d_a2);               // 2nd joint's angle

    double s_a3 = r3*sin(delta - alpha2)/L3;
    double c_a3 = ((z4-L1)*(z4-L1) + x4*x4 -L3*L3 -L2*L2)/(2*L2*L3);
    double alpha3 = atan2(s_a3, c_a3);                          // 3rd joint's angle

    double alpha4 = (1-k)*(M_PI/2 + theta) - (alpha2 + alpha3); // 4th joint's angle
    double alpha5 = k*(M_PI/2 + theta);                         // 5th joint's angle

    if(isnan(alpha2) || isnan(alpha3)) // NaN value check
      return false;

    angles.push_back(alpha1);
    angles.push_back(alpha2);
    angles.push_back(alpha3);
    angles.push_back(alpha4);
    angles.push_back(alpha5);
    angles.push_back(th6);

}

bool Kinematics::solveIK(double x, double y, double z,
                        double theta, double grip_angle,
                        double result[5])
{
  if((grip_angle >= 0.0 && grip_angle <= M_PI/2) &&
     (theta >= -M_PI/2  && theta <= M_PI))
  {
    double k = 0.9;

    double alpha1 = atan2(y, x);                                // 1st joint's angle

    double x5 = x - (L5 + F0*cos(grip_angle) + F1) * cos(theta)*cos(alpha1);
    double z5 = z + (L5 + F0*cos(grip_angle) + F1) * sin(theta);

    double x4 = x5 - L4 * sin((1-k)*(M_PI/2 + theta))*cos(alpha1); // some conditions for IK simplification
    double z4 = z5 - L4 * cos((1-k)*(M_PI/2 + theta));

    double r4 = sqrt(x4*x4 + z4*z4);
    double beta = atan2(z4, x4);

    double r3 = sqrt(L1*L1 + r4*r4 - 2*L1*r4*sin(beta));
    double delta = atan2(x4, z4 - L1);

    double s_d_a2 = 1/(2*r3*L2)*sqrt(((z4-L1)*(z4-L1) - (L3-L2)*(L3-L2)) * ((L2+L3)*(L2+L3) - (z4-L1)*(z4-L1)));
    double c_d_a2 = (L2*L2 + r3*r3 - L3*L3)/(2*L2*r3);

    double alpha2 = delta - atan2(s_d_a2, c_d_a2);               // 2nd joint's angle

    double s_a3 = r3*sin(delta - alpha2)/L3;
    double c_a3 = ((z4-L1)*(z4-L1) + x4*x4 -L3*L3 -L2*L2)/(2*L2*L3);
    double alpha3 = atan2(s_a3, c_a3);                          // 3rd joint's angle

    double alpha4 = (1-k)*(M_PI/2 + theta) - (alpha2 + alpha3); // 4th joint's angle
    double alpha5 = k*(M_PI/2 + theta);                         // 5th joint's angle

    if(isnan(alpha2) || isnan(alpha3)) // NaN value check
      return false;

    result[0] = alpha1;
    result[1] = alpha2;
    result[2] = alpha3;
    result[3] = alpha4;
    result[4] = alpha5;

    return true;

  }

  return false;

}

bool Kinematics::solveIK(double x, double y, double z,
                         double theta,
                         double grip_angle,
                         std::vector<double> &res_ang)
{

  if((grip_angle >= 0.0 && grip_angle <= M_PI/2) &&
     (theta >= -M_PI/2  && theta <= M_PI))
  {
    double k = 0.9;

    double r = sqrt(x*x + y*y);
    double alpha1 = atan2(y, x);                                // 1st joint's angle

    double x5 = r - (L5 + F0*cos(grip_angle) + F1) * cos(theta)*cos(alpha1);
    double z5 = z + (L5 + F0*cos(grip_angle) + F1) * sin(theta);

    double x4 = x5 - L4 * sin((1-k)*(M_PI/2 + theta))*cos(alpha1); // some conditions for IK simplification
    double z4 = z5 - L4 * cos((1-k)*(M_PI/2 + theta));

    double r4 = sqrt(x4*x4 + z4*z4);
    double beta = atan2(z4, x4);

    double r3 = sqrt(L1*L1 + r4*r4 - 2*L1*r4*sin(beta));
    double delta = atan2(x4, z4 - L1);

    double s_d_a2 = 1/(2*r3*L2)*sqrt(((z4-L1)*(z4-L1) - (L3-L2)*(L3-L2)) * ((L2+L3)*(L2+L3) - (z4-L1)*(z4-L1)));
    double c_d_a2 = (L2*L2 + r3*r3 - L3*L3)/(2*L2*r3);

    double alpha2 = delta - atan2(s_d_a2, c_d_a2);               // 2nd joint's angle

    double s_a3 = r3*sin(delta - alpha2)/L3;
    double c_a3 = ((z4-L1)*(z4-L1) + x4*x4 -L3*L3 -L2*L2)/(2*L2*L3);
    double alpha3 = atan2(s_a3, c_a3);                          // 3rd joint's angle

    double alpha4 = (1-k)*(M_PI/2 + theta) - (alpha2 + alpha3); // 4th joint's angle
    double alpha5 = k*(M_PI/2 + theta);                         // 5th joint's angle

    if(isnan(alpha2) || isnan(alpha3)) // NaN value check
      return false;

    res_ang.push_back(alpha1);
    res_ang.push_back(alpha2);
    res_ang.push_back(alpha3);
    res_ang.push_back(alpha4);
    res_ang.push_back(alpha5);
    res_ang.push_back(grip_angle);

    return true;

  }

  return false;

}

void Kinematics::getJAngles(double &th1, double &th2, double &th3,
                            double &th4, double &th5, double &th6)
{
  th1 = this->th1;
  th2 = this->th2;
  th3 = this->th3;
  th4 = this->th4;
  th5 = this->th5;
  th6 = this->th6;

}

std::vector<double> Kinematics::getJAngles()
{
  std::vector<double> result;

  result.push_back(this->th1);
  result.push_back(this->th2);
  result.push_back(this->th3);
  result.push_back(this->th4);
  result.push_back(this->th5);
  result.push_back(this->th6);

  return result;
}

TransMatrix Kinematics::getMatrix()
{

  return ee_pose;

}
