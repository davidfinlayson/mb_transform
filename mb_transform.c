/*!
 * \file    	mb_transform.c
 * \brief   	Coordinate transformation library for MB System
 * \author	David Finlayson
 * \date	March 28, 2014
 *
 * A simple coodinate transformation library for MB System. This library is
 * designed to perform the simple affine transformations needed during the
 * routine processing of multibeam data. It supports ridged translations and
 * rotations about an arbitrary origin and includes a SLERP function for
 * smoothly interpolating orientations.
 *
 * The library (currently) uses a left-handed coordinate system with the
 * following axis:
 *
 * - x-axis positive to the right
 * - y-axis positive up
 * - z-axis positive forward
 *
 * - Rotation about the x-axis (pitch) is positive nose down
 * - Rotation about the y-axis (heading) is positive nose right
 * - Rotation about the z-axis (bank) is positive starboard up
 *
 * The library is based on a 4x3 transformation matrix (mb_matrix) which can
 * be computed very efficiently. Rotations and translations can be applied in
 * arbitrary order which is necessary to support sonars which do not use
 * the heading-pitch-bank system (aka roll-pitch-yaw).
 *
 * Adapted from:
 *
 * Dunn, Fletcher and Parberry, Ian (2002) 3D Math Primer for Graphics
 * and Game Development. WordWare Publishing, Inc., Sudbury, MA
 *
 * Mak, Ronald (2003) The Java Programmer's Guide to Numerical Computing.
 * Prentice Hall PTR, Upper Saddle River, NJ
 */

#include <math.h>
#include "mb_transform.h"

static double wrap_pi(double theta);
static void set_identity(mb_matrix *ma);
static void concat_transform(mb_matrix *a, mb_matrix *b);

/*!
 * \brief initialize a 4x3 transformation matrix
 * \param m a transformation matrix
 */
void mb_init_transform(mb_matrix *m)
{
	set_identity(m);
}


/*!
 * \brief apply the transformations defined in m to vector p
 * \param m transformation matrix
 * \param p vector representing a point
 */
void mb_transform(const mb_matrix *m, mb_vector *p)
{
	double x, y, z;

	x = p->x * m->m11 + p->y * m->m21 + p->z * m->m31 + m->tx;
	y = p->x * m->m12 + p->y * m->m22 + p->z * m->m32 + m->ty;
	z = p->x * m->m13 + p->y * m->m23 + p->z * m->m33 + m->tz;

	p->x = x;
	p->y = y;
	p->z = z;
}


/*!
 * \brief apply a translation action to the transformation matrix
 * \param dx translate along x-axis (positive to right)
 * \param dy translate along y-axis (positive up)
 * \param dz translate along z-axis (positive forward)
 */
void mb_concat_translation(mb_matrix *m, double dx, double dy, double dz)
{
	mb_matrix translate;

	set_identity(&translate);
	translate.tx = dx;
	translate.ty = dy;
	translate.tz = dz;
	concat_transform(m, &translate);
}


/*!
 * \brief apply a rotation about the x-axis to the transformation matrix
 * \param pitch angle of declination in radians (positive nose down)
 */
void mb_concat_rotate_x(mb_matrix *m, double pitch)
{
	double sint, cost;
	mb_matrix rotate;

	sint = sin(pitch);
	cost = cos(pitch);

	set_identity(&rotate);
	rotate.m22 = cost;
	rotate.m23 = sint;
	rotate.m32 = -sint;
	rotate.m33 = cost;
	concat_transform(m, &rotate);
}


/*!
 * \brief apply a roation about the y-axis to the transformation matrix
 * \param heading angle of heading in radians (positive to right)
 */
void mb_concat_rotate_y(mb_matrix *m, double heading)
{
	double sint, cost;
	mb_matrix rotate;

	sint = sin(heading);
	cost = cos(heading);

	set_identity(&rotate);
	rotate.m11 = cost;
	rotate.m13 = -sint;
	rotate.m31 = sint;
	rotate.m33 = cost;
	concat_transform(m, &rotate);
}


/*!
 * \brief apply rotation about the z-axis to the transformtion matrix
 * \param bank angle of bank in radians (positive starboard up)
 */
void mb_concat_rotate_z(mb_matrix *m, double bank)
{
	double sint, cost;
	mb_matrix rotate;

	sint = sin(bank);
	cost = cos(bank);

	set_identity(&rotate);
	rotate.m11 = cost;
	rotate.m12 = sint;
	rotate.m21 = -sint;
	rotate.m22 = cost;
	concat_transform(m, &rotate);
}


/*!
 * \brief Setup the quaternion to perform an object->inertial rotation, given
 * 	the orientation in Euler angle format
 * \param orientation in Euler angle format
 * \param q object->inertial quaternion
 */
void mb_angles_to_quat(const mb_angles *orientation, mb_quaternion *q)
{
	double sp, sb, sh;
	double cp, cb, ch;

	sp = sin(0.5 * orientation->pitch);
	cp = cos(0.5 * orientation->pitch);
	sb = sin(0.5 * orientation->bank);
	cb = cos(0.5 * orientation->bank);
	sh = sin(0.5 * orientation->heading);
	ch = cos(0.5 * orientation->heading);

	q->w = ch * cp * cb + sh * sp * sb;
	q->x = ch * sp * cb + sh * cp * sb;
	q->y = -ch * sp * sb + sh * cp * cb;
	q->z = -sh * sp * cb + ch * cp * sb;
}


/*!
 * \brief Setup the Euler angles, given an object -> inertial rotation
 *	quaternion
 * \param q object->inertial quaternion
 * \param orientation orienation in Euler angle format
 */
void mb_quat_to_angles(const mb_quaternion *q, mb_angles *orientation)
{
	double sp;

	sp = -2.0 * (q->y * q->z - q->w * q->x);

	if (fabs(sp) > 0.9999)
		{
		orientation->pitch = kPiOver2 * sp;
		orientation->heading = atan2(-q->x * q->z + q->w * q->y,
			0.5 - q->y * q->y - q->z * q->z);
		orientation->bank = 0.0;
		}
	else
		{
		orientation->pitch = asin(sp);
		orientation->heading = atan2(q->x * q->z + q->w * q->y,
			0.5 - q->x * q->x - q->y * q->y);
		orientation->bank = atan2(q->x * q->y + q->w * q->z,
			0.5 - q->x * q->x - q->z * q->z);
		}
}


/*!
 * \brief Spherical linear interpolation
 * \param q0 starting orientation as an object->inertial quaternion
 * \param q1 ending orientation as an object->inertial quaternion
 * \param t interpolation fraction ranging from 0 to 1 (corresponging to q0 and
 * q1, respectively)
 */
void mb_slerp(
	const mb_quaternion *q0, const mb_quaternion *q1, double t,
	mb_quaternion *q)
{
	double q1w, q1x, q1y, q1z;
	double k0, k1;
	double omega, cosOmega, sinOmega, oneOverSinOmega;

	/* Check for out of range parameter and return edge points if so */
	if (t <= 0.0)
		{
		*q = *q0;
		}
	else if (t >= 1.0)
		{
		*q = *q1;
		}
	else
		{
		/* Compute "cosine of angle between quaternions" using dot product */
		cosOmega = q0->w * q1->w + q0->x * q1->x + q0->y * q1->y + q0->z *
			q1->z;

		/* Chose q or -q to rotate using the acute angle */
		q1w = q1->w;
		q1x = q1->x;
		q1y = q1->y;
		q1z = q1->z;
		if (cosOmega < 0.0)
			{
			q1w = -q1w;
			q1x = -q1x;
			q1y = -q1y;
			q1z = -q1z;
			cosOmega = -cosOmega;
			}

		/* We should have two unit quaternions, so dot should be <= 1.0
		   assert(cosOmega < 1.1); */

		/* Compute interpolation fraction */
		if (cosOmega > 0.9999)
			{
			/* very close - just use linear interpolation */
			k0 = 1.0 - t;
			k1 = t;
			}
		else
			{
			sinOmega = sqrt(1.0 - cosOmega * cosOmega);
			omega = atan2(sinOmega, cosOmega);
			oneOverSinOmega = 1.0 / sinOmega;
			k0 = sin((1.0 - t) * omega) * oneOverSinOmega;
			k1 = sin(t * omega) * oneOverSinOmega;
			}

		/* Interpolate */
		q->x = k0 * q0->x + k1 * q1x;
		q->y = k0 * q0->y + k1 * q1y;
		q->z = k0 * q0->z + k1 * q1z;
		q->w = k0 * q0->w + k1 * q1w;
		}
}	/* mb_slerp */




/*!
 * \brief wrap an angle in range -pi ... pi by adding the correct multiple of
 *	2pi
 * \param theta angle to wrap (radians)
 */
static double wrap_pi(double theta)
{
	theta += kPi;
	theta -= floor(theta * k1Over2Pi) * k2Pi;
	theta -= kPi;
	return (theta);
}


/*!
 * \brief set/reset an mb_matrix back to the identity matrix
 * \param m transformation matrix to reset
 */
static void set_identity(mb_matrix *m)
{
	/* rotation matrix */
	m->m11 = 1.0; m->m12 = 0.0; m->m13 = 0.0;
	m->m21 = 0.0; m->m22 = 1.0; m->m23 = 0.0;
	m->m31 = 0.0; m->m32 = 0.0; m->m33 = 1.0;

	/* translation vector */
	m->tx = 0.0; m->ty = 0.0; m->tz = 0.0;
}


/*!
 * /brief concatinates a transformation matrix b onto an existing transformation
 *	matrix a
 * /param a transformation matrix a
 * /param a transformation matrix b
 */
static void concat_transform(
	mb_matrix *a, mb_matrix *b)
{
	mb_matrix r;

	/* rotation matrix */
	r.m11 = a->m11 * b->m11 + a->m12 * b->m21 + a->m13 * b->m31;
	r.m12 = a->m11 * b->m12 + a->m12 * b->m22 + a->m13 * b->m32;
	r.m13 = a->m11 * b->m13 + a->m12 * b->m23 + a->m13 * b->m33;

	r.m21 = a->m21 * b->m11 + a->m22 * b->m21 + a->m23 * b->m31;
	r.m22 = a->m21 * b->m12 + a->m22 * b->m22 + a->m23 * b->m32;
	r.m23 = a->m21 * b->m13 + a->m22 * b->m23 + a->m23 * b->m33;

	r.m31 = a->m31 * b->m11 + a->m32 * b->m21 + a->m33 * b->m31;
	r.m32 = a->m31 * b->m12 + a->m32 * b->m22 + a->m33 * b->m32;
	r.m33 = a->m31 * b->m13 + a->m32 * b->m23 + a->m33 * b->m33;

	/* translation vector */
	r.tx = a->tx * b->m11 + a->ty * b->m21 + a->tz * b->m31 + b->tx;
	r.ty = a->tx * b->m12 + a->ty * b->m22 + a->tz * b->m32 + b->ty;
	r.tz = a->tx * b->m13 + a->ty * b->m23 + a->tz * b->m33 + b->tz;

	/* copy the results back into first matrix */
	a->m11 = r.m11; a->m12 = r.m12; a->m13 = r.m13;
	a->m21 = r.m21; a->m22 = r.m22; a->m23 = r.m23;
	a->m31 = r.m31; a->m32 = r.m32; a->m33 = r.m33;
	a->tx = r.tx; a->ty = r.ty; a->tz = r.tz;
}


