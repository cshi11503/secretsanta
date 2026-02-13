/**
   EN.601.463/663
   Assignment #1 

   Cartesian trajectory generation
   
 */
#include "assignment1.hpp"
#include <math.h>
#include <cmath>

// Compute the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematics( double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4] ){
  
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      E[r][c] = 0.0;
  // TODO
  // Fill the values of the forward kinematics (homogeneous matrix E)
  // Rotation

    E[0][0] = sin(q2 + q3 + q4)*cos(q1)*sin(q6)- cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5));

    E[0][1] = sin(q6)*(sin(q1)*sin(q5)+ cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6);

    E[0][2] = cos(q2 + q3 + q4)*cos(q1)*sin(q5)- cos(q5)*sin(q1);

    E[1][0] = cos(q6)*(cos(q1)*sin(q5)- cos(q2 + q3 + q4)*cos(q5)*sin(q1))+ sin(q2 + q3 + q4)*sin(q1)*sin(q6);

    E[1][1] = sin(q2 + q3 + q4)*cos(q6)*sin(q1)- sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1));

    E[1][2] = cos(q1)*cos(q5)+ cos(q2 + q3 + q4)*sin(q1)*sin(q5);


    E[2][0] = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6);

    E[2][1] = cos(q2 + q3 + q4)*cos(q6)- sin(q2 + q3 + q4)*cos(q5)*sin(q6);

    E[2][2] = -sin(q2 + q3 + q4)*sin(q5);
    
    // Translation

    E[0][3] =
        (17.0*cos(q1)*cos(q2))/40.0
        - (133.0*sin(q1))/1000.0
        - (11.0*cos(q5)*sin(q1))/40.0
        + (11.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5))/40.0
        - (cos(q2 + q3)*cos(q1)*sin(q4))/10.0
        - (sin(q2 + q3)*cos(q1)*cos(q4))/10.0
        + (49.0*cos(q1)*cos(q2)*cos(q3))/125.0
        - (49.0*cos(q1)*sin(q2)*sin(q3))/125.0;


    E[1][3] =
        (133.0*cos(q1))/1000.0
        + (11.0*cos(q1)*cos(q5))/40.0
        + (17.0*cos(q2)*sin(q1))/40.0
        - (49.0*sin(q1)*sin(q2)*sin(q3))/125.0
        + (11.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5))/40.0
        - (cos(q2 + q3)*sin(q1)*sin(q4))/10.0
        - (sin(q2 + q3)*cos(q4)*sin(q1))/10.0
        + (49.0*cos(q2)*cos(q3)*sin(q1))/125.0;


    E[2][3] =
        (sin(q2 + q3)*sin(q4))/10.0
        - (17.0*sin(q2))/40.0
        - sin(q5)*((11.0*cos(q2 + q3)*sin(q4))/40.0
        + (11.0*sin(q2 + q3)*cos(q4))/40.0)
        - (cos(q2 + q3)*cos(q4))/10.0
        - (49.0*sin(q2 + q3))/125.0
        + 163.0/1000.0;


    // bottom row
    E[3][0] = 0.0;
    E[3][1] = 0.0;
    E[3][2] = 0.0;
    E[3][3] = 1.0;
}

// Compute the inverse of the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematicsInverse( double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4] ){

  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      E[r][c] = 0.0;

  // TODO
  // Fill the values of the inverse of the forward kinematics (homogeneous matrix E)
  double E[4][4];

    // First compute forward kinematics
    ForwardKinematics(q1, q2, q3, q4, q5, q6, E);

    // Extract rotation transpose
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            E_inv[r][c] = E[c][r];   // transpose

    // Compute -R^T * t
    for(int r = 0; r < 3; r++)
    {
        E_inv[r][3] =
            -( E_inv[r][0] * E[0][3]
             + E_inv[r][1] * E[1][3]
             + E_inv[r][2] * E[2][3] );
    }

    // Bottom row
    E_inv[3][0] = 0.0;
    E_inv[3][1] = 0.0;
    E_inv[3][2] = 0.0;
    E_inv[3][3] = 1.0;
}

// Compute the Adjoint transformation inverse matrix
// input E: the rotation/translation between the base and the hand frame
//          (as computed by the forward kinematics)
// output Ad: the 6x6 adjoint transformation inverse matrix
void AdjointTransformationInverse( double E[4][4], double Ad[6][6] ){

  for(int r=0; r<6; r++)
    for(int c=0; c<6; c++)
      Ad[r][c] = 0.0;  
  // TODO
  // Compute the Adjoint Transformation Inverse A^-1
  // Extract R^T directly
    double RT[3][3];

    for(int r=0; r<3; r++)
        for(int c=0; c<3; c++)
            RT[r][c] = E[c][r];   // transpose

    // Extract translation t
    double tx = E[0][3];
    double ty = E[1][3];
    double tz = E[2][3];

    // Build t_hat
    double t_hat[3][3];

    t_hat[0][0] = 0.0;   t_hat[0][1] = -tz;  t_hat[0][2] =  ty;
    t_hat[1][0] =  tz;   t_hat[1][1] = 0.0;  t_hat[1][2] = -tx;
    t_hat[2][0] = -ty;   t_hat[2][1] =  tx;  t_hat[2][2] = 0.0;

    // Fill upper-left block: R^T
    for(int r=0; r<3; r++)
        for(int c=0; c<3; c++)
            Ad[r][c] = RT[r][c];

    // Compute upper-right block: -R^T * t_hat
    for(int r=0; r<3; r++)
        for(int c=0; c<3; c++)
        {
            Ad[r][c+3] = 0.0;
            for(int k=0; k<3; k++)
                Ad[r][c+3] -= RT[r][k] * t_hat[k][c];
        }

    // Lower-right block: R^T
    for(int r=0; r<3; r++)
        for(int c=0; c<3; c++)
            Ad[r+3][c+3] = RT[r][c];
  

}
void matMult(double A[4][4], double B[4][4], double C[4][4])
{
    for(int r=0;r<4;r++)
        for(int c=0;c<4;c++)
        {
            C[r][c]=0.0;
            for(int k=0;k<4;k++)
                C[r][c]+=A[r][k]*B[k][c];
        }
}

void getZ(double T[4][4], double z[3])
{
    z[0] = T[0][2];
    z[1] = T[1][2];
    z[2] = T[2][2];
}

void getP(double T[4][4], double p[3])
{
    p[0] = T[0][3];
    p[1] = T[1][3];
    p[2] = T[2][3];
}

void cross(double a[3], double b[3], double c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}


// Compute and return the Jacobian of the robot given the current joint positions
// input: the joints angles
// output: the 6x6 Jacobian
void Jacobian( double q1, double q2, double q3, double q4, double q5, double q6, double J[6][6] ){
  
  for(int r=0; r<6; r++)
    for(int c=0; c<6; c++)
      J[r][c] = 0.0;
  // TODO
  // Fill the values of the Jacobian matrix J
  // =========================
    // Build each transform
    // =========================

    double E_ws[4][4] = {
        {-cos(q1),  sin(q1), 0, 0},
        {-sin(q1), -cos(q1), 0, 0},
        {0,0,1,0.163},
        {0,0,0,1}
    };

    double E_su[4][4] = {
        {cos(q2), -sin(q2), 0, 0},
        {0,0,-1,0},
        {sin(q2), cos(q2),0,0},
        {0,0,0,1}
    };

    double E_uf[4][4] = {
        {cos(q3), -sin(q3),0,-0.425},
        {sin(q3), cos(q3),0,0},
        {0,0,1,0},
        {0,0,0,1}
    };
    double E_fw1[4][4] = {
        {cos(q4), -sin(q4),0,-0.392},
        {sin(q4), cos(q4),0,0},
        {0,0,1,0.133},
        {0,0,0,1}
    };

    double E_w1w2[4][4] = {
        {cos(q5), -sin(q5),0,0},
        {0,0,-1,-0.1},
        {sin(q5), cos(q5),0,0},
        {0,0,0,1}
    };

    double E_w2w3[4][4] = {
        {cos(q6), -sin(q6),0,0},
        {0,0,1,0.1},
        {-sin(q6), -cos(q6),0,0},
        {0,0,0,1}
    };
    double E_w3g[4][4] = {
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,0.175},
        {0,0,0,1}
    };

    // =========================
    // Compute cumulative transforms
    // =========================

    double E_wu[4][4], E_wf[4][4], E_ww1[4][4];
    double E_ww2[4][4], E_ww3[4][4], E_wg[4][4];

    matMult(E_ws,E_su,E_wu);
    matMult(E_wu,E_uf,E_wf);
    matMult(E_wf,E_fw1,E_ww1);
    matMult(E_ww1,E_w1w2,E_ww2);
    matMult(E_ww2,E_w2w3,E_ww3);
    matMult(E_ww3,E_w3g,E_wg);

    // =========================
    // Extract pe
    // =========================
    double pe[3];
    getP(E_wg,pe);

    // List of transforms up to each joint
    double* Tlist[6] = {
        &E_ws[0][0],
        &E_wu[0][0],
        &E_wf[0][0],
        &E_ww1[0][0],
        &E_ww2[0][0],
        &E_ww3[0][0]
    };

    double (*T[6])[4] = {E_ws,E_wu,E_wf,E_ww1,E_ww2,E_ww3};

    for(int i=0;i<6;i++)
    {
        double zi[3], pi[3], diff[3], Jv[3];

        getZ(T[i],zi);
        getP(T[i],pi);

        diff[0]=pe[0]-pi[0];
        diff[1]=pe[1]-pi[1];
        diff[2]=pe[2]-pi[2];

        cross(zi,diff,Jv);

        J[0][i]=Jv[0];
        J[1][i]=Jv[1];
        J[2][i]=Jv[2];
        J[3][i]=zi[0];
        J[4][i]=zi[1];
        J[5][i]=zi[2];
    }


}

