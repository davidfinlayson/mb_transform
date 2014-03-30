#include <math.h>
#include <stdio.h>
#include "mb_transform.h"

static void print_vector(mb_vector *p);
static void print_matrix(mb_matrix *m);
static void print_angles(mb_angles *a);

int main(void)
{
	mb_vector p;
	mb_matrix m;
	mb_angles a1, a2;
	mb_quaternion q1, q2;
	int i;
	
	/* build up a series of transformations */
	mb_init_transform(&m);
	mb_concat_translation(&m, 10, 0, 0);
	mb_concat_rotate_z(&m, -10 * DTR);	

	/* create a point */
	p.x = 0.0; p.y = 0.0; p.z = 0.0;
	printf("Point before translation:\n");
	print_vector(&p);
	
	/* perform the transformation */
	mb_transform(&m, &p);
	printf("Point after transformation:\n");
	print_vector(&p);
	
	/* interpolate attitude */
	
	/* starting angle */
	a1.heading = 350.0 * DTR;
	a1.pitch = -10.0 * DTR;
	a1.bank = +10.0 * DTR;
	mb_angles_to_quat(&a1, &q1);
	
	/* ending angle */
	a2.heading = 10.0 * DTR;
	a2.pitch = +10.0 * DTR;
	a2.bank = -10.0 * DTR;
	mb_angles_to_quat(&a2, &q2);
	
	printf("\nInterpolate attitude:\n");
	print_angles(&a1);
	for(i = 1; i < 10; i++)
	{
		mb_quaternion q;
		mb_angles a;
		double t;
		
		/* calculate a fraction t = 0 ... 1 */
		t = (double)i/10.0;
		
		mb_slerp(&q1, &q2, t, &q);
		mb_quat_to_angles(&q, &a);
		print_angles(&a);
	}
	print_angles(&a2);
	
	return 0;
}

static void print_vector(mb_vector *p)
{
	printf("[%.3f,%.3f,%.3f]\n", p->x, p->y, p->z);
}

static void print_matrix(mb_matrix *m)
{
	printf("| %.3f %.3f %.3f |\n", m->m11, m->m12, m->m13);
	printf("| %.3f %.3f %.3f |\n", m->m21, m->m22, m->m23);
	printf("| %.3f %.3f %.3f |\n", m->m31, m->m32, m->m33);
	printf("| %.3f %.3f %.3f |\n", m->tx, m->ty, m->tz);
}

static void print_angles(mb_angles *a)
{
	printf("heading = %.3f pitch = %.3f bank = %.3f\n", a->heading * RTD, a->pitch * RTD,
		a->bank * RTD);
}

