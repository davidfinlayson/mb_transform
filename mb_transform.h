/*!
 * \file 	mb_transform.h
 * \brief 	Coordinate transformation library for MB System
 * \author	David Finlayson
 * \date	March 28, 2014
 */

#define DTR ( M_PI/180.0 )	/*!< degrees to radians */ 
#define RTD ( 180.0/M_PI )	/*!< radians to degrees */

#define kPi ( M_PI )				/*!< pi */
#define k2Pi ( kPi * 2.0 )			/*!< 2 * pi */
#define kPiOver2 ( kPi / 2.0 )		/*!< pi over 2 */
#define k1OverPi ( 1.0 / kPi )		/*!< 1 over pi */
#define k1Over2Pi ( 1.0 / k2Pi )	/*!< 1 over 2 pi */

/*! a simple 3D vector and/or point */
typedef struct vector_struct
{
	double x;
	double y;
	double z;
} mb_vector;

/*! a 4x3 transformation matrix */
typedef struct matrix_struct
{
	double m11, m12, m13;
	double m21, m22, m23;
	double m31, m32, m33;
	double tx, ty, tz;
} mb_matrix;

/*! a heading-pitch-bank Euler angle triple */
typedef struct angles_struct
{
	double heading;
	double pitch;
	double bank;
} mb_angles;

/*! A quaternion for the purpose of representing an angular displacement */
typedef struct quaternion_struct
{
	double w;
	double x;
	double y;
	double z;
} mb_quaternion;

void mb_init_transform(mb_matrix *m);
void mb_concat_translation(mb_matrix *m, double dx, double dy, double dz);
void mb_concat_rotate_x(mb_matrix *m, double pitch);
void mb_concat_rotate_y(mb_matrix *m, double heading);
void mb_concat_rotate_z(mb_matrix *m, double bank);
void mb_transform(const mb_matrix *m, mb_vector *p);

void mb_angles_to_quat(const mb_angles *orientation, mb_quaternion *q);
void mb_quat_to_angles(const mb_quaternion *q, mb_angles *orientation);
void mb_slerp(
	const mb_quaternion *q0, const mb_quaternion *q1, double t,
	mb_quaternion *q);

