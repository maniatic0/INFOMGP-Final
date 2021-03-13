#ifndef CONSTRAINTS_HEADER_FILE
#define CONSTRAINTS_HEADER_FILE

using namespace Eigen;
using namespace std;

typedef enum ConstraintType{DISTANCE, COLLISION} ConstraintType;   //You can expand it for more constraints
typedef enum ConstraintEqualityType{EQUALITY, INEQUALITY_GREATER, INEQUALITY_SMALLER} ConstraintEqualityType;

//there is such constraints per two variables that are equal. That is, for every attached vertex there are three such constraints for (x,y,z);
class Constraint{
public:
  
  int m1, m2;                     //Two participating meshes (can be the same)  - auxiliary data for users (constraint class shouldn't use that)
  int v1, v2;                     //Two vertices from the respective meshes - auxiliary data for users (constraint class shouldn't use that)
  double invMass1, invMass2;       //inverse masses of two bodies
  double refValue;                //Reference values to use in the constraint, when needed (like distance)
  RowVector3d refVector;             //Reference vector when needed (like vector)
  double CRCoeff;                 //extra velocity bias
  ConstraintType constraintType;  //The type of the constraint, and will affect the value and the gradient. This SHOULD NOT change after initialization!
  ConstraintEqualityType constraintEqualityType;  //whether the constraint is an equality or an inequality
  
  Constraint(const ConstraintType _constraintType, const ConstraintEqualityType _constraintEqualityType, const int& _m1, const int& _v1, const int& _m2, const int& _v2, const double& _invMass1, const double& _invMass2, const RowVector3d& _refVector, const double& _refValue, const double& _CRCoeff):constraintType(_constraintType), constraintEqualityType(_constraintEqualityType), m1(_m1), v1(_v1), m2(_m2), v2(_v2), invMass1(_invMass1), invMass2(_invMass2),  refValue(_refValue), CRCoeff(_CRCoeff){
    refVector=_refVector;
  }
  
  ~Constraint(){}
  
  
  
  //computes the impulse needed for all particles to resolve the velocity constraint, and corrects the velocities accordingly.
  //The velocities are a vector (vCOM1, w1, vCOM2, w2) in both input and output.
  //returns true if constraint was already valid with "currVelocities", and false otherwise (false means there was a correction done)
  //currCOMPositions is a 2x3 matrix, where each row is per one of the sides of the constraints; the rest of the relevant variables are similar, and so should the outputs be resized.
  bool resolveVelocityConstraint(const MatrixXd& currCOMPositions, const MatrixXd& currVertexPositions, const MatrixXd& currCOMVelocities, const MatrixXd& currAngularVelocities, const Matrix3d& invInertiaTensor1, const Matrix3d& invInertiaTensor2, MatrixXd& correctedCOMVelocities, MatrixXd& correctedAngularVelocities, double tolerance){
      MatrixXd invMassMatrix = MatrixXd::Zero(12,12);
      for (size_t i = 0; i < 3; i++)
      {
          invMassMatrix(i, i) = invMass1;
          invMassMatrix(i+6, i+6) = invMass2;
          for (size_t j = 0; j < 3; j++)
          {
              invMassMatrix(i + 3, j + 3) = invInertiaTensor1(i, j);
              invMassMatrix(i + 9, j + 9) = invInertiaTensor2(i, j);
          }
      }

  	Vector3d r0 = currVertexPositions.row(0) - currCOMPositions.row(0);
    Vector3d r1 = currVertexPositions.row(1) - currCOMPositions.row(1);
  	Vector3d n = (currVertexPositions.row(0) - currVertexPositions.row(1)).normalized();
  	Vector3d l0 = r0.cross( n );
    Vector3d l1 = r1.cross( n );

    VectorXd vel(12);
    VectorXd jT(12);
    for (size_t i = 0; i < 3; i++)
    {
        vel(i) = currCOMVelocities(0, i);
        vel(i+3) = currAngularVelocities(0, i);
        vel(i+6) = currCOMVelocities(1, i);
        vel(i + 9) = currAngularVelocities(1, i);

    	jT(i) = n(i);
    	jT(i+3) = l0(i);
    	jT(i+6) = -n(i);
    	jT(i+9) = -l1(i);
    }

    correctedCOMVelocities = currCOMVelocities;
    correctedAngularVelocities = currAngularVelocities;

  	double jv = jT.dot(vel);
  	if ( constraintEqualityType == ConstraintEqualityType::EQUALITY && std::abs(jv) <= tolerance || constraintEqualityType != ConstraintEqualityType::EQUALITY/*||  
        constraintEqualityType == ConstraintEqualityType::INEQUALITY_GREATER && jv >= -tolerance || 
        constraintEqualityType == ConstraintEqualityType::INEQUALITY_SMALLER && jv <= tolerance*/ ) { //TODO why does this not work??

#if 0
        cout << "J " << jT << endl;
        cout << "pos " << vel << endl;

        if (constraintEqualityType == ConstraintEqualityType::EQUALITY)
        {
            cout << "Equality Constraint ";
            cout << "Tolerance reached: " << std::abs(jv) << " <= " << tolerance << endl;

        }
        else
        {
            cout << "Inequality Constraint ";
            cout << "Tolerance reached: " << jv << " >= " << -tolerance << endl;
        }
#endif // 0

  		return true;
  	}

  	double lamb = -(1.0 + CRCoeff) * jv / ( jT.transpose().dot(static_cast<VectorXd>(invMassMatrix * jT)) );

    VectorXd delV = lamb * static_cast<VectorXd>( invMassMatrix * jT );
    
    for (size_t i = 0; i < 3; i++)
    {
        correctedCOMVelocities(0, i) += delV(i);
        correctedCOMVelocities(1, i) += delV(i + 6);
        correctedAngularVelocities(0, i) += delV(i + 3);
		correctedAngularVelocities(1, i) += delV(i + 9);
    }

  	return false;

    /**************
     TODO: write velocity correction procedure:
     1. If the velocity Constraint is satisfied up to tolerate ("abs(Jv)<=tolerance"), set corrected values to original ones and return true
     
     2. Otherwise, correct linear and angular velocities as learnt in class.
     
     Note to differentiate between different constraint types; for inequality constraints you don't do anything unless it's unsatisfied.
     ***************/
  }
  
  //projects the position unto the constraint
  //returns true if constraint was already valid with "currPositions"
  bool resolvePositionConstraint(const MatrixXd& currCOMPositions, const MatrixXd& currConstPositions, MatrixXd& correctedCOMPositions, double tolerance){
      MatrixXd invMassMatrix = MatrixXd::Zero(6, 6);
      for (size_t i = 0; i < 3; i++)
      {
          invMassMatrix(i, i) = invMass1;
          invMassMatrix(i + 3, i + 3) = invMass2;
      }


      Vector3d p0 = currConstPositions.row(0);
      Vector3d p1 = currConstPositions.row(1);
      Vector3d n = (p0 - p1).normalized();

      VectorXd pos(6);
      VectorXd jT(6);
      for (size_t i = 0; i < 3; i++)
      {
          pos(i) = p0(i);
          pos(i + 3) = p1(i);


          jT(i) = n(i);
          jT(i+3) = -n(i);
      }

      correctedCOMPositions = currCOMPositions;

      double jp = jT.dot(pos) - refValue;
      if (constraintEqualityType == ConstraintEqualityType::EQUALITY && std::abs(jp) <= tolerance ||
          constraintEqualityType == ConstraintEqualityType::INEQUALITY_GREATER && jp >= -tolerance || 
          constraintEqualityType == ConstraintEqualityType::INEQUALITY_SMALLER && jp <= tolerance) {

#if 0
          cout << "J " << jT << endl;
          cout << "pos " << pos << endl;

          if (constraintEqualityType == ConstraintEqualityType::EQUALITY)
          {
              cout << "Equality Constraint ";
              cout << "Tolerance reached: " << std::abs(jp) << " <= " << tolerance << endl;

          }
          else
          {
              cout << "Inequality Constraint ";
              cout << "Tolerance reached: " << jp << " >= " << -tolerance << endl;
          }
#endif // 0

          return true;
      }

      double lamb = - jp / (jT.transpose().dot(static_cast<VectorXd>(invMassMatrix * jT)));

      VectorXd delP = lamb * static_cast<VectorXd>(invMassMatrix * jT);

      for (size_t i = 0; i < 3; i++)
      {
          correctedCOMPositions(0, i) += delP(i);
          correctedCOMPositions(1, i) += delP(i + 3);
      }

      return false;
    /**************
     TODO: write position correction procedure:
     1. If the position Constraint is satisfied up to tolerate ("abs(C(p)<=tolerance"), set corrected values to original ones and return true
     
     2. Otherwise, correct COM position as learnt in class. Note that since this is a linear correction, correcting COM position == correcting all positions the same offset. the currConstPositions are used to measure the constraint, and the COM values are corrected accordingly to create the effect.
     
     Note to differentiate between different constraint types; for inequality constraints you don't do anything unless it's unsatisfied.
     ***************/
  }
};



#endif /* constraints_h */
