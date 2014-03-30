/**************************************************************************
   mb_transform.c - Coordinate transformation library

   David Finlayson
   March 28, 2014

   Adapted from:

   Dunn, Fletcher and Parberry, Ian (2002) 3D Math Primer for Graphics
   and Game Development. WordWare Publishing, Inc., Sudbury, MA

   Mak, Ronald (2003) The Java Programmer's Guide to Numerical Computing.
   Prentice Hall PTR, Upper Saddle River, NJ

 *************************************************************************/
 #include <math.h>
 #include "mb_transform.h"

static double wrapPi(double theta);
static void set_identity(mb_matrix *ma);
static void concat_transform(mb_matrix *a, mb_matrix *b);

/*------------------------------------------------------------------*/
void mb_init_transform(mb_matrix *m)
{
	set_identity(m);
}
/*------------------------------------------------------------------*/
void mb_transform(mb_matrix *m, mb_vector *p)
{
	double x, y, z;

	x = p->x * m->m11 + p->y * m->m21 + p->z * m->m31 + m->tx;
	y = p->x * m->m12 + p->y * m->m22 + p->z * m->m32 + m->ty;
	z = p->x * m->m13 + p->y * m->m23 + p->z * m->m33 + m->tz;

	p->x = x;
	p->y = y;
	p->z = z;
}
/*------------------------------------------------------------------*/
void mb_concat_translation(mb_matrix *m, double dx, double dy, double dz)
{
	mb_matrix translate;

	set_identity(&translate);
	translate.tx = dx;
	translate.ty = dy;
	translate.tz = dz;
	concat_transform(m, &translate);
}


/*------------------------------------------------------------------*/
void mb_concat_rotate_x(mb_matrix *m, double theta)
{
	double sint, cost;
	mb_matrix rotate;

	sint = sin(theta);
	cost = cos(theta);

	set_identity(&rotate);
	rotate.m22 = cost;
	rotate.m23 = sint;
	rotate.m32 = -sint;
	rotate.m33 = cost;
	concat_transform(m, &rotate);
}
/*------------------------------------------------------------------*/
void mb_concat_rotate_y(mb_matrix *m, double theta)
{
	double sint, cost;
	mb_matrix rotate;

	sint = sin(theta);
	cost = cos(theta);

	set_identity(&rotate);
	rotate.m11 = cost;
	rotate.m13 = -sint;
	rotate.m31 = sint;
	rotate.m33 = cost;
	concat_transform(m, &rotate);
}
/*------------------------------------------------------------------*/
void mb_concat_rotate_z(mb_matrix *m, double theta)
{
	double sint, cost;
	mb_matrix rotate;

	sint = sin(theta);
	cost = cos(theta);

	set_identity(&rotate);
	rotate.m11 = cost;
	rotate.m12 = sint;
	rotate.m21 = -sint;
	rotate.m22 = cost;
	concat_transform(m, &rotate);
}
/*------------------------------------------------------------------*/
void mb_angles_to_quat(mb_angles *orientation, mb_quaternion *q)
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
/*------------------------------------------------------------------*/
void mb_quat_to_angles(mb_quaternion *q, mb_angles *orientation)
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
/*------------------------------------------------------------------*/
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

		/* We should have two unit quaternions, so dot should be <= 1.0 */
		/* assert(cosOmega < 1.1); */

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

static double wrapPi(double theta)
{
	theta += kPi;
	theta -= floor(theta * k1Over2Pi) * k2Pi;
	theta -= kPi;
	return theta;
}
	
static void set_identity(mb_matrix *ma)
{
	ma->m11 = 1.0; ma->m12 = 0.0; ma->m13 = 0.0;
	ma->m21 = 0.0; ma->m22 = 1.0; ma->m23 = 0.0;
	ma->m31 = 0.0; ma->m32 = 0.0; ma->m33 = 1.0;
	ma->tx = 0.0; ma->ty = 0.0; ma->tz = 0.0;
}
static void concat_transform(mb_matrix *a, mb_matrix *b)
{
	mb_matrix r;

	/* calculate the upper transformation portion of matrix */
	r.m11 = a->m11 * b->m11 + a->m12 * b->m21 + a->m13 * b->m31;
	r.m12 = a->m11 * b->m12 + a->m12 * b->m22 + a->m13 * b->m32;
	r.m13 = a->m11 * b->m13 + a->m12 * b->m23 + a->m13 * b->m33;

	r.m21 = a->m21 * b->m11 + a->m22 * b->m21 + a->m23 * b->m31;
	r.m22 = a->m21 * b->m12 + a->m22 * b->m22 + a->m23 * b->m32;
	r.m23 = a->m21 * b->m13 + a->m22 * b->m23 + a->m23 * b->m33;

	r.m31 = a->m31 * b->m11 + a->m32 * b->m21 + a->m33 * b->m31;
	r.m32 = a->m31 * b->m12 + a->m32 * b->m22 + a->m33 * b->m32;
	r.m33 = a->m31 * b->m13 + a->m32 * b->m23 + a->m33 * b->m33;

	/* calculate the lower translation portion of matrix */
	r.tx = a->tx * b->m11 + a->ty * b->m21 + a->tz * b->m31 + b->tx;
	r.ty = a->tx * b->m12 + a->ty * b->m22 + a->tz * b->m32 + b->ty;
	r.tz = a->tx * b->m13 + a->ty * b->m23 + a->tz * b->m33 + b->tz;

	/* copy the results back into matrix a */
	a->m11 = r.m11; a->m12 = r.m12; a->m13 = r.m13;
	a->m21 = r.m21; a->m22 = r.m22; a->m23 = r.m23;
	a->m31 = r.m31; a->m32 = r.m32; a->m33 = r.m33;
	a->tx = r.tx; a->ty = r.ty; a->tz = r.tz;
}

