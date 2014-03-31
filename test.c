#include <math.h>
#include <stdio.h>
#include "mb_transform.h"


static void print_vector(mb_vector *p);
static void print_matrix(mb_matrix *m);
static void print_angles(mb_angles *a);
void mb_lever(double sonar_offset_x, double sonar_offset_y,
	double sonar_offset_z, double nav_offset_x, double nav_offset_y,
	double nav_offset_z, double vru_offset_x, double vru_offset_y,
	double vru_offset_z, double vru_pitch, double vru_roll,
	double *lever_x, double *lever_y, double *lever_z);
void mb_lever_new(mb_vector *sonar_offset, mb_vector *nav_offset,
	mb_vector *vru_offset, mb_angles *orientation, mb_vector *lever);


int main(void)
{
	mb_vector p;
	mb_matrix m;
	mb_angles a1, a2;
	mb_quaternion q1, q2;
	int i;
	
	mb_vector sonar_offset, nav_offset, vru_offset;
	mb_vector sonar_pos, nav_pos;
	mb_angles orientation;
	
	/* Example 1a - Simple translation
         *
         * We have a point at (10, 0, 0) and we want to
         * to move it to (0, 10, 0).
         */

	printf("\nEXAMPLE 1a\n");
	p.x = 10; p.y = 0; p.z = 0;
	printf("Starting point: ");
	print_vector(&p);

	mb_init_transform(&m);
	mb_concat_translation(&m, -10, 10, 0); /* move 10 to the left, +10 up */

	mb_transform(&m, &p);	
	printf("Ending vector: ");
	print_vector(&p);


	/* Example 1b - Simple translation
         *
         * Same problem as above, but now we will break it into
         * two seperate movements. The only additional computational cost
         * is during mb_concat_translation. The exact same number of operations
         * are used during mb_transform.
         */

	printf("\nEXAMPLE 1b\n");
	p.x = 10.0; p.y = 0.0; p.z = 0.0;
	printf("Starting vector: ");
	print_vector(&p);
	
	mb_init_transform(&m);
	mb_concat_translation(&m, -10, 0, 0); /* move 10 to the left */
	mb_concat_translation(&m, 0, 10, 0); /* move 10 up */

	mb_transform(&m, &p);	
	printf("Ending vector: ");
	print_vector(&p);

	/* Example 2 - Simple rotation
         *
         * Now we will rotate a point (10, 0, 0) about two axis in turn.
         * first we rotate 90 degrees about the y axis (heading)
         * then we rotate 90 degrees about the x axis (pitch)
         * the resulting point will be (0, -10, 0)
         */

	printf("\nEXAMPLE 2:\n");
	p.x = 10.0; p.y = 0.0; p.z = 0.0;
	printf("Starting vector: ");
	print_vector(&p);
	
	mb_init_transform(&m);
	mb_concat_rotate_y(&m, -90 * DTR); /* rotate 90 degrees nose to the left */
	mb_concat_rotate_x(&m, 90 * DTR); /* rotate 90 degrees nose down */

	mb_transform(&m, &p);	
	printf("Ending vector: ");
	print_vector(&p);

	/* Example 3 - Realistic rotation about an arbitrary origin (the vru)
         *
         * Say our boat has the following configuration:
         */

	printf("\nExample 3:\n");

	/* location of position antenna */
	nav_offset.x = -10;	// 10 meters to the left of the reference point
	nav_offset.y = 10;	// 10 meters above the reference point
	nav_offset.z = 10;	//  10 mters forward of the reference point
	printf("Positon of the nav relative to the reference point: ");
	print_vector(&nav_offset);

	/* location of the vru */
	vru_offset.x = 1;	// 1 meter to the right of the reference point
	vru_offset.y = -1;	// 1 meter below the reference point
	vru_offset.z = 0;	// 0 meters forward of the reference point
	printf("Position of the vru relative to the reference point: ");
	print_vector(&vru_offset);

	/* sonar transducer */
	sonar_offset.x = 5;		// 5 meters to the right of the reference point
	sonar_offset.y = -10;	// 10 meters below the reference point
	sonar_offset.z = -3;		// 3 meters behind the reference point
	printf("Position of the sonar relative to the reference point: ");
	print_vector(&sonar_offset);

	/* orienation of the vessel */
	orientation.heading = 0 * DTR;
	orientation.pitch = 0 * DTR;
	orientation.bank = 10 * DTR;

	/*
         * What is the x, y, z offset of the transducer relative to the navigation antenna?
         */

	/* Step 1: move the point of rotation (the vru) to the origin */
	mb_init_transform(&m);
	mb_concat_translation(&m, -vru_offset.x, -vru_offset.y, -vru_offset.z);

	/* Step 2: rotate the vessel into inertial alignment */
	mb_concat_rotate_z(&m, orientation.bank);
	mb_concat_rotate_x(&m, orientation.pitch);
	mb_concat_rotate_y(&m, orientation.heading);

	/* Step 3: calculate the new positions of the nav and sonar relative to the imu */
	nav_pos = nav_offset;
	mb_transform(&m, &nav_pos);
	
	sonar_pos = sonar_offset;
	mb_transform(&m, &sonar_pos);

	printf("Inertial nav position: ");
	print_vector(&nav_pos);

	printf("Inertial sonar position: ");
	print_vector(&sonar_pos);

	printf("Nav - Sonar Offsets:\n");
	printf("nav.x - sonar.x: %.2f\n", sonar_pos.x - nav_pos.x);
	printf("nav.y - sonar.y: %.2f\n", sonar_pos.y - nav_pos.y);
	printf("nav.z 0 sonar.z: %.2f\n", sonar_pos.z - nav_pos.z);	
	
	
	/* Example 4 - interpolate attitude
         *
         * This example shows how to interpolate attitude smoothly.
         * It easily crosses the -360 to +0 boundary, handles gimbel
         * lock (hope your boat is never gimbel locked!) and works
         * with very little code.
         */

	printf("\nEXAMPLE 4:\n");
	
	/* starting angle */
	a1.heading = 350.0 * DTR;
	a1.pitch = -10.0 * DTR;
	a1.bank = +10.0 * DTR;
	printf("Starting orienation: ");
	print_angles(&a1);
	mb_angles_to_quat(&a1, &q1);
	
	/* ending angle */
	a2.heading = 10.0 * DTR;
	a2.pitch = +10.0 * DTR;
	a2.bank = -10.0 * DTR;
	printf("Ending orienation: ");
	print_angles(&a2);
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

