/**************************************************************************
  mb_transform.h - Coordinate transformation library
  
  David Finlayson
  March 28, 2014

  Adapted from:

  Dunn, Fletcher and Parberry, Ian (2002) 3D Math Primer for Graphics
  and Game Development. WordWare Publishing, Inc., Sudbury, MA

  Mak, Ronald (2003) The Java Programmer's Guide to Numerical Computing.
  Prentice Hall PTR, Upper Saddle River, NJ

  ***************************************************************************/
#define DTR ( M_PI/180.0 )
#define RTD ( 180.0/M_PI )

#define kPi ( M_PI )
#define k2Pi ( kPi * 2.0 )
#define kPiOver2 ( kPi / 2.0 )
#define k1OverPi ( 1.0 / kPi )
#define k1Over2Pi ( 1.0 / k2Pi )

typedef struct vector_struct
{
	double x;
	double y;
	double z;
} mb_vector;

typedef struct matrix_struct
{
	double m11, m12, m13;
	double m21, m22, m23;
	double m31, m32, m33;
	double tx, ty, tz;
} mb_matrix;

typedef struct angles_struct
{
	double heading;
	double pitch;
	double bank;
} mb_angles;

typedef struct quaternion_struct
{
	double w;
	double x;
	double y;
	double z;
} mb_quaternion;

void mb_init_transform(mb_matrix *m);
void mb_concat_translation(mb_matrix *m, double dx, double dy, double dz);
void mb_concat_rotate_x(mb_matrix *m, double theta);
void mb_concat_rotate_y(mb_matrix *m, double theta);
void mb_concat_rotate_z(mb_matrix *m, double theta);
void mb_transform(mb_matrix *m, mb_vector *p);
void mb_angles_to_quat(mb_angles *orientation, mb_quaternion *q);
void mb_quat_to_angles(mb_quaternion *q, mb_angles *orientation);
void mb_slerp(
	const mb_quaternion *q0, const mb_quaternion *q1, double t,
	mb_quaternion *q);

