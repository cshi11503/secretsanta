/**
   EN.601.463/663
   Assignment #1 

   Cartesian trajectory generation
   
 */
#include "assignment1.hpp"
#include <math.h>

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
  double T[4][4];

    // First compute forward kinematics
    ForwardKinematics(q1,q2,q3,q4,q5,q6,T);

    // Transpose rotation
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            E[i][j] = T[j][i];

    // Compute -R^T p
    for(int i=0;i<3;i++)
    {
        E[i][3] = 0.0;
        for(int j=0;j<3;j++)
            E[i][3] -= E[i][j] * T[j][3];
    }

    //bottom row
    E[3][0] = 0.0;
    E[3][1] = 0.0;
    E[3][2] = 0.0;
    E[3][3] = 1.0;
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
  double R[3][3];
    double p[3];

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
            R[i][j] = E[i][j];

        p[i] = E[i][3];
    }

    // Compute R transpose
    double RT[3][3];
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            RT[i][j] = R[j][i];

    // Compute skew(p)
    double skew[3][3];
    skew[0][0] = 0;       skew[0][1] = -p[2];  skew[0][2] =  p[1];
    skew[1][0] = p[2];    skew[1][1] = 0;      skew[1][2] = -p[0];
    skew[2][0] = -p[1];   skew[2][1] = p[0];   skew[2][2] = 0;

    // Compute -RT * skew(p)
    double bottomLeft[3][3];
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            bottomLeft[i][j] = 0.0;
            for(int k=0;k<3;k++)
                bottomLeft[i][j] -= RT[i][k] * skew[k][j];
        }

    // Assemble Adjoint inverse matrix

    // Top-left = R^T
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            Ad[i][j] = RT[i][j];

    // Top-right = 0 (already zero)

    // Bottom-left = -R^T [p]x
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            Ad[i+3][j] = bottomLeft[i][j];

    // Bottom-right = R^T
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            Ad[i+3][j+3] = RT[i][j];

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
  // --- Compute intermediate transforms ---
    double E_ws[4][4];
    double E_wu[4][4];
    double E_wf[4][4];
    double E_ww1[4][4];
    double E_ww2[4][4];
    double E_ww3[4][4];
    double E_wg[4][4];

    // You must already have these functions implemented
    Transform_ws(q1, E_ws);
    Transform_wu(q1, q2, E_wu);
    Transform_wf(q1, q2, q3, E_wf);
    Transform_ww1(q1, q2, q3, q4, E_ww1);
    Transform_ww2(q1, q2, q3, q4, q5, E_ww2);
    Transform_ww3(q1, q2, q3, q4, q5, q6, E_ww3);

    ForwardKinematics(q1,q2,q3,q4,q5,q6,E_wg);

    // Put transforms in array
    double (*Elist[6])[4] = {E_ws, E_wu, E_wf, E_ww1, E_ww2, E_ww3};

    // End-effector position
    double pe[3] = {
        E_wg[0][3],
        E_wg[1][3],
        E_wg[2][3]
    };

    for(int i=0;i<6;i++)
    {
        // Origin of frame i
        double pi[3] = {
            Elist[i][0][3],
            Elist[i][1][3],
            Elist[i][2][3]
        };

        // zi axis = third column of rotation
        double zi[3] = {
            Elist[i][0][2],
            Elist[i][1][2],
            Elist[i][2][2]
        };

        // pe - pi
        double diff[3] = {
            pe[0] - pi[0],
            pe[1] - pi[1],
            pe[2] - pi[2]
        };

        // Linear velocity part: zi × diff
        J[0][i] = zi[1]*diff[2] - zi[2]*diff[1];
        J[1][i] = zi[2]*diff[0] - zi[0]*diff[2];
        J[2][i] = zi[0]*diff[1] - zi[1]*diff[0];

        // Angular velocity part
        J[3][i] = zi[0];
        J[4][i] = zi[1];
        J[5][i] = zi[2];
    }
}

