#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <time.h>

#include <gdk/gdkkeysyms.h>
#include <gtk/gtk.h>

#define DEFAULT_WIDTH 800
#define DEFAULT_HEIGHT 800

static void do_drawing(cairo_t *, GtkWidget *widget);

struct ball_face;

struct ball
{
	double x;
	double y;
	unsigned int radius;

	double v_x;
	double v_y;

	double angle;
	double v_angle;

	double moment_of_inertia;

	struct ball_face *face;
};

double delta = 0.01; /* seconds */

unsigned int width = DEFAULT_WIDTH;
unsigned int height = DEFAULT_HEIGHT;

unsigned int radius_min = 5;
unsigned int radius_max = 10;

unsigned int v_max = 100;
unsigned int v_min = 1;

unsigned int v_angle_min = 1;
unsigned int v_angle_max = 100;

struct ball *balls = 0;
unsigned int n_balls = 10;

double g_y = 20;
double g_x = 0;

double clear_alpha = 1.0;

// Structure for the main polygon
struct
{
	int count;
	double coordx[100];
	double coordy[100];
	double v_x;
	double v_y;
	/* coordinates of the center of mass */
	double x;
	double y;
	// only for the sphere
	unsigned int radius;
	double angle;
	// angular velocity
	double v_angle_x;
	double v_angle_y;
	// mass of the polygon
	int mass;
	double moment_inertia_x;
	double moment_inertia_y;

} glob;
// Keep trak wether the figure is completed
// (The user has finished to draw)
int glob_complete = 0;

// A boolean value to check whether the drawed
// polygon is simple or not
int simple_polygon = 1;

// Assign a random velocity to a ball
// The values for v_x and v_y should be inside a range (v_min, v_max)
void random_velocity(struct ball *p)
{
	double r2;
	do
	{
		p->v_x = v_min + rand() % (v_max + 1 - v_min);
		p->v_y = v_min + rand() % (v_max + 1 - v_min);
		r2 = p->v_x * p->v_x + p->v_y * p->v_y;
	} while (r2 > v_max * v_max || r2 < v_min * v_min);
}

// Initialise the state of the balls
// Position each ball randomly on the canvas
// Assign a random velocity to each ball
void balls_init_state()
{
	srand(time(NULL));
	static const unsigned int border = 10;
	unsigned int w = width < 2 * border ? 1 : width - 2 * border;
	unsigned int h = height < 2 * border ? 1 : height - 2 * border;

	for (unsigned int i = 0; i < n_balls; ++i)
	{
		balls[i].x = border + rand() % w;
		balls[i].y = border + rand() % h;
		random_velocity(balls + i);
		if (rand() % 2)
			balls[i].v_x = -balls[i].v_x;
		if (rand() % 2)
			balls[i].v_y = -balls[i].v_y;
		balls[i].radius = radius_min + rand() % (radius_max + 1 - radius_min);
		unsigned int v_angle_360 = (v_angle_min + rand() % (v_angle_max + 1 - v_angle_min)) % 360;
		balls[i].v_angle = 2 * M_PI * v_angle_360 / 360;
		balls[i].angle = (rand() % 360) * 2 * M_PI / 360;
		// Calculate the moment of inertia
		balls[i].moment_of_inertia = M_PI * pow((balls[i].radius * 2), 4) / 64;
	}
}

// Check collisions between two balls,
// and update their state based on that
void ball_collision(struct ball *p, struct ball *q)
{
	double dx = q->x - p->x;
	double dy = q->y - p->y;
	double d2 = dx * dx + dy * dy;
	double r = p->radius + q->radius;
	if (d2 <= r * r)
	{
		double dv_x = q->v_x - p->v_x;
		double dv_y = q->v_y - p->v_y;

		double mp = p->radius * p->radius;
		double mq = q->radius * q->radius;

		double f = dv_x * dx + dv_y * dy;

		if (f < 0)
		{
			f /= d2 * (mp + mq);
			p->v_x += 2 * mq * f * dx;
			p->v_y += 2 * mq * f * dy;

			q->v_x -= 2 * mp * f * dx;
			q->v_y -= 2 * mp * f * dy;
		}
	}
}

#if 0
static void tangential_friction1(double u, double p, double r, double * v, double * q) {
    static const double a = 0.0;

    /*
     *                  2   2      2                 2   2  2
     *     sqrt((6 - 2 a ) u  + 4 a  p r u + (3 - 2 a ) p  r ) + a u - a p r
     * v = -----------------------------------------------------------------
     *                                     3
     *
     *                  2   2      2                 2   2  2
     *     sqrt((6 - 2 a ) u  + 4 a  p r u + (3 - 2 a ) p  r ) - 2 a u + 2 a p r
     * q = ---------------------------------------------------------------------
     *                                      3 r
     */
    double a2 = a*a;
    double u2 = u*u;
    double p2 = p*p;
    double r2 = r*r;
    double sr = sqrt((6 - 2*a2)*u2 + 4*a2*p*r*u + (3 - 2*a2)*p2*r2);
    *v = (sr + a*u - a*p*r)/3;
    *q = (sr - 2*a*u + 2*a*p*r)/(3*r);
}

static void tangential_friction2(double u, double p, double r, double * v, double * q) {
    static const double a = 1.0;

    /*
     *                  2   2      2                 2   2  2
     *     sqrt((6 - 2 a ) u  + 4 a  p r u + (3 - 2 a ) p  r ) + a u + a p r
     * v = -----------------------------------------------------------------
     *                                     3
     *
     *                  2   2      2                 2   2  2
     *     sqrt((6 - 2 a ) u  + 4 a  p r u + (3 - 2 a ) p  r ) - 2 a u - 2 a p r
     * q = ---------------------------------------------------------------------
     *                                      3 r
     */
    double a2 = a*a;
    double u2 = u*u;
    double p2 = p*p;
    double r2 = r*r;
    double sr = sqrt((6 - 2*a2)*u2 + 4*a2*p*r*u + (3 - 2*a2)*p2*r2);
    *v = (sr + a*u + a*p*r)/3;
    *q = (sr - 2*a*u - 2*a*p*r)/(3*r);
}

static void tangential_friction3(double u, double p, double r, double * v_ptr, double * q_ptr) {
    static const double a = 0.9;
    double v, q;
    v = (1-a)*u + a*p*r;
    /*                    2   2       2                        2   2  2
     *     sqrt((4 a - 2 a ) u  + (4 a  - 4 a) p r u + (1 - 2 a ) p  r )
     * q = -------------------------------------------------------------
     *                                   r
     */
    q = sqrt(2*a*(2 - a)*u*u + 4*a*(a - 1)*p*r*u + (1 - 2*a*a)*p*p*r*r)/r;
    if (p*r > v)
	q = -q;

    *v_ptr = v;
    *q_ptr = q;
}
#endif

// Update the state of the balls
void ball_update_state(struct ball *p)
{
	p->x += delta * p->v_x + delta * delta * g_x / 2.0;
	p->v_x += delta * g_x;

	p->y += delta * p->v_y + delta * delta * g_y / 2.0;
	p->v_y += delta * g_y;

	if (p->x + p->radius > width)
	{ /* right wall */
		if (p->v_x > 0)
		{
			p->x -= p->x + p->radius - width;
			p->v_x = -p->v_x;
#if 0
	    /* tangential friction */
	    tangential_friction(p->v_y, -p->v_angle, p->radius, &(p->v_y), &(p->v_angle));
#endif
		}
	}
	else if (p->x < p->radius)
	{ /* left wall */
		if (p->v_x < 0)
		{
			p->x += p->radius - p->x;
			p->v_x = -p->v_x;
#if 0
	    /* tangential friction */
	    tangential_friction(p->v_y, p->v_angle, p->radius, &(p->v_y), &(p->v_angle));
#endif
		}
	}

	if (p->y + p->radius > height)
	{ /* bottom wall */
		if (p->v_y > 0)
		{
			p->y -= p->y + p->radius - height;
			p->v_y = -p->v_y;
#if 0
	    /* tangential friction */
	    tangential_friction3(p->v_x, p->v_angle, p->radius, &(p->v_x), &(p->v_angle));
#endif
		}
	}
	else if (p->y < p->radius)
	{ /* top wall */
		if (p->v_y < 0)
		{
			p->y += p->radius - p->y;
			p->v_y = -p->v_y;
#if 0
	    /* tangential friction */
	    tangential_friction(p->v_x, -p->v_angle, p->radius, &(p->v_x), &(p->v_angle));
	    p->v_angle = -p->v_angle;
#endif
		}
	}
	p->angle += delta * p->v_angle;
	while (p->angle >= 2 * M_PI)
		p->angle -= 2 * M_PI;
	while (p->angle < 0)
		p->angle += 2 * M_PI;
}

void reposition_within_borders()
{
	for (int i = 0; i < n_balls; ++i)
	{
		struct ball *p = balls + i;
		if (p->x < p->radius)
			p->x = p->radius;
		else if (p->x + p->radius > width)
			p->x = width - p->radius;
		if (p->y < p->radius)
			p->y = p->radius;
		else if (p->y + p->radius > height)
			p->y = height - p->radius;
	}
}

// For each balls update its state
void movement_and_borders()
{
	for (int i = 0; i < n_balls; ++i)
		ball_update_state(balls + i);
}

/* Collision check with index
 */
struct rectangle
{
	double min_x; /* left */
	double min_y; /* bottom */
	double max_x; /* right */
	double max_y; /* top */
};

struct bt_node
{
	struct ball *ball;
	struct rectangle r;
	struct bt_node *left;
	struct bt_node *right;
};

struct bt_node *c_index = 0;

static struct bt_node *c_index_init_node(struct bt_node *n, struct ball *b)
{
	n->ball = b;
	n->r.min_x = b->x - b->radius;
	n->r.min_y = b->y - b->radius;
	n->r.max_x = b->x + b->radius;
	n->r.max_y = b->y + b->radius;
	n->left = 0;
	n->right = 0;
	return n;
}

static void c_index_add_ball(struct bt_node *n, const struct ball *b)
{
	if (n->r.min_x > b->x - b->radius)
		n->r.min_x = b->x - b->radius;
	if (n->r.min_y > b->y - b->radius)
		n->r.min_y = b->y - b->radius;
	if (n->r.max_x < b->x + b->radius)
		n->r.max_x = b->x + b->radius;
	if (n->r.max_y < b->y + b->radius)
		n->r.max_y = b->y + b->radius;
}

static void c_index_insert(struct bt_node *t, struct bt_node *n, struct ball *b)
{
	double w = width;
	double h = height;
	double ref_x = 0.0;
	double ref_y = 0.0;
	c_index_init_node(n, b);
	for (;;)
	{
		c_index_add_ball(t, b);
		if (w > h)
		{ /* horizontal split */
			if (b->x <= t->ball->x)
			{
				if (t->left)
				{
					w = t->ball->x - ref_x;
					t = t->left;
				}
				else
				{
					t->left = n;
					return;
				}
			}
			else
			{
				if (t->right)
				{
					w -= t->ball->x - ref_x;
					ref_x = t->ball->x;
					t = t->right;
				}
				else
				{
					t->right = n;
					return;
				}
			}
		}
		else
		{ /* vertical split */
			if (b->y <= t->ball->y)
			{
				if (t->left)
				{
					h = t->ball->y - ref_y;
					t = t->left;
				}
				else
				{
					t->left = n;
					return;
				}
			}
			else
			{
				if (t->right)
				{
					h -= t->ball->y - ref_y;
					ref_y = t->ball->y;
					t = t->right;
				}
				else
				{
					t->right = n;
					return;
				}
			}
		}
	}
}

void c_index_build()
{
	c_index_init_node(c_index, balls);
	for (int i = 1; i < n_balls; ++i)
		c_index_insert(c_index, c_index + i, balls + i);
}

struct bt_node **c_index_stack = 0;
unsigned int c_index_stack_top = 0;

static void c_index_stack_clear()
{
	c_index_stack_top = 0;
}

static void c_index_stack_push(struct bt_node *n)
{
	c_index_stack[c_index_stack_top++] = n;
}

static struct bt_node *c_index_stack_pop()
{
	if (c_index_stack_top > 0)
		return c_index_stack[--c_index_stack_top];
	else
		return 0;
}

static int c_index_ball_in_rectangle(const struct bt_node *n, const struct ball *b)
{
	return n->r.min_x <= b->x + b->radius && n->r.max_x >= b->x - b->radius && n->r.min_y <= b->y + b->radius && n->r.max_y >= b->y - b->radius;
}

static int c_index_must_check(const struct bt_node *n, const struct ball *b)
{
	return n != 0 && n->ball < b && c_index_ball_in_rectangle(n, b);
}

// For each ball check if there is a collision with
// the other balls
void c_index_check_collisions()
{
	for (struct ball *b = balls + 1; b < balls + n_balls; ++b)
	{
		c_index_stack_clear();
		struct bt_node *n = c_index;
		do
		{
			ball_collision(n->ball, b);
			if (c_index_must_check(n->left, b))
			{
				if (c_index_must_check(n->right, b))
					c_index_stack_push(n->right);
				n = n->left;
			}
			else if (c_index_must_check(n->right, b))
			{
				n = n->right;
			}
			else
			{
				n = c_index_stack_pop();
			}
		} while (n);
	}
}

// Initialise and allocate the memory for the c_index
int c_index_init()
{
	if (!c_index)
		c_index = malloc(sizeof(struct bt_node) * n_balls);
	if (!c_index)
		return 0;
	if (!c_index_stack)
		c_index_stack = malloc(sizeof(struct bt_node *) * n_balls);
	if (!c_index_stack)
		return 0;
	return 1;
}

// Dealloc the memory allocated for the c_index
void c_index_destroy()
{
	if (c_index)
		free(c_index);
	if (c_index_stack)
		free(c_index_stack);
	c_index = 0;
	c_index_stack = 0;
}

/* Trivial collision check
 */


// Method for doing the dot product
double dot_product(double v[], double u[])
{
	return (v[0] * u[0]) + (v[1] * u[1]);
}

// Method for cross product
double crossProduct(double vect_A[], double vect_B[])

{
	// fprintf(stderr, "%f \n",  result[0]);
	return (vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]);
}




// Calculate the area of a tringle in a geometric coordinates
double area_triangle(double x_a, double y_a, double x_b, double y_b)
{
	double a = x_a * (y_b - glob.y);
	double b = x_b * (glob.y - y_a);
	double c = glob.x * (y_a - y_b);
	return fabs(a + b + c) / 2;
}

// Calculate the moment of inertia of the polygon
void moment_of_inertia()
{
	double sum_x = 0;
	double sum_y = 0;
	for (size_t i = 0; i < glob.count; i++)
	{
		int j = i + 1;
		if (j == glob.count)
		{
			j = 0;
		}
		double a = area_triangle(glob.coordx[i], glob.coordy[i], glob.coordx[j], glob.coordy[j]);
		sum_x += (pow(glob.coordy[i], 2) + (glob.coordy[i] * glob.coordy[j]) + pow(glob.coordy[j], 2)) * a;
		sum_y += (pow(glob.coordx[i], 2) + (glob.coordx[i] * glob.coordx[j]) + pow(glob.coordx[j], 2)) * a;
	}
	glob.moment_inertia_x = sum_x / 12;
	glob.moment_inertia_y = sum_y / 12;
	// fprintf(stderr, "%f \n", glob.moment_inertia_x);
}

// void moment_of_inertia()
// {
// 	double area = 0;
//     double center[] = {0.0, 0.0};
//     double mmoi = 0;

//     int prev = glob.count-1;
//     for (int index = 0; index < glob.count; index++)
//     {
//         double a[] = {glob.coordx[prev], glob.coordy[prev]};
//         double b[] ={glob.coordx[index], glob.coordy[index]};

//         double area_step = crossProduct(a, b)/2;
//         double center_step[] = {(a[0]+b[0])/3, (a[1]+b[1])/3};
//         double mmoi_step = area_step*(dot_product(a, a)+dot_product(b, b)+dot_product(a, b))/6;

//         center[0] = (center[0]*area + center_step[0] * area_step)/(area + area_step);
// 		center[1] = (center[1]*area + center_step[1] * area_step)/(area + area_step);
//         area += area_step;
//         mmoi += mmoi_step;

//         prev = index;
//     }

//     double density = glob.mass/area;
//     mmoi *= density;
//     mmoi -= glob.mass * dot_product(center, center);

// 	glob.moment_inertia_x  = glob.moment_inertia_y = mmoi;
	
// }



// Calculate the right respond witht the collision between balls and polygon
void collision_ball_polygon_response(double x, double y, double nb[], struct ball* p)
{
	double vel_vector_polygon[] = {glob.v_x, glob.v_y};
	double dis_vector_polygon[] = {x - glob.x, y - glob.y};

	double vel_vector_ball[] = {p->v_x, p->v_y};



	double magnitude = sqrt(nb[0] * nb[0] + nb[1] * nb[1]);
	nb[0] /= magnitude;
	nb[1] /= magnitude;

	double first_term_x = -2 * (dot_product(vel_vector_polygon, nb) - dot_product(vel_vector_ball, nb) + glob.v_angle_x * (crossProduct(dis_vector_polygon, nb) - p->v_angle * (crossProduct(dis_vector_polygon, nb))));
	double second_term_x = 1 / glob.mass + (pow(crossProduct(dis_vector_polygon, nb), 2) / glob.moment_inertia_x);

	double first_term_y = -2 * (dot_product(vel_vector_polygon, nb) - dot_product(vel_vector_ball, nb) + glob.v_angle_y * (crossProduct(dis_vector_polygon, nb) - p->v_angle * (crossProduct(dis_vector_polygon, nb))));
	double second_term_y = 1 / glob.mass + (pow(crossProduct(dis_vector_polygon, nb), 2) / glob.moment_inertia_y);

	double j_x = first_term_x / second_term_x;
	double j_y = first_term_y / second_term_y;

	double j_vector[] = {j_x, j_y};

	glob.v_x += (1 / glob.mass) * j_x;
	glob.v_y += (1 / glob.mass) * j_y;
	// fprintf(stderr, "%f \n", glob.v_x);

	glob.v_angle_x += crossProduct(dis_vector_polygon, j_vector) / glob.moment_inertia_x;
	glob.v_angle_y += crossProduct(dis_vector_polygon, j_vector) / glob.moment_inertia_y;
}

// Method for checking and updating if a sphere intersects the polygon
void check_polygon_intersection()
{
	for (int i = 0; i < n_balls; ++i)
	{
		double sin_a = sin(glob.angle);
		double cos_a = cos(glob.angle);
		struct ball *p = balls + i;
		for (int j = 0; j < glob.count; j++)
		{
			double current_x = glob.x + cos_a * glob.coordx[j] - sin_a * glob.coordy[j];
			double current_y = glob.y + sin_a * glob.coordx[j] + cos_a * glob.coordy[j];



			// calc delta distance: source point to line start
			double dx = p->x - current_x;
			double dy = p->y - current_y;
			int next = j + 1;

			if (j == glob.count - 1)
			{
				next = 0;
			}
			double next_x = glob.x + cos_a * glob.coordx[next] - sin_a * glob.coordy[next];
			double next_y = glob.y + sin_a * glob.coordx[next] + cos_a * glob.coordy[next];
			// calc delta distance: line start to end
			double dxx = next_x - current_x;
			double dyy = next_y - current_y;

			// Calc position on line normalized between 0.00 & 1.00
			// == dot product divided by delta line distances squared
			double t = (dx * dxx + dy * dyy) / (dxx * dxx + dyy * dyy);

			// calc nearest pt on line
			double x = current_x + dxx * t;
			double y = current_y + dyy * t;

			// clamp results to being on the segment
			if (t < 0)
			{
				x = current_x;
				y = current_y;
			}
			if (t > 1)
			{
				x = next_x;
				y = next_y;
			}
			double a = p->x - x;
			double b = p->y - y;
			int collide = 0;
			if ((a * a + b * b) < p->radius * p->radius)
			{
				collide = 1;
			}

			if (collide)
			{
				double nb[2];
				nb[0] = glob.x - x;
				nb[1] = y;
				// collision_ball_polygon_response(x, y, nb, p);
				// double first_term = -2 * ( );
				// double second_term = (1/glob.mass) + 1 + ;
				// double j = first_term / second_term;
				// https://fotino.me/2d-rigid-body-collision-response/
			}
		}
	}
};

void check_collisions_simple()
{
	for (int i = 0; i < n_balls; ++i)
		for (int j = i + 1; j < n_balls; ++j)
			ball_collision(balls + i, balls + j);
}

void check_collisions_with_index()
{
	c_index_build();
	c_index_check_collisions();
	// Check the collision with the polygon
	check_polygon_intersection();
}

// Perchè esiste questo metodo?
void (*check_collisions)() = 0;

void update_state()
{
	if (check_collisions)
		check_collisions();
	movement_and_borders();
}

/* Graphics System
 */

GtkWidget *window;
GtkWidget *canvas;

int gravity_vector_countdown = 0;
int gravity_vector_init = 300;

void draw_gravity_vector(cairo_t *cr)
{
	if (gravity_vector_countdown != 0)
	{
		cairo_new_path(cr);
		cairo_move_to(cr, width / 2, height / 2);
		cairo_line_to(cr, width / 2 + g_x, height / 2 + g_y);
		cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
		cairo_set_line_width(cr, 1.0);
		cairo_stroke(cr);
		cairo_arc(cr, width / 2 + g_x, height / 2 + g_y, 3, 0, 2 * M_PI);
		cairo_fill(cr);
		if (gravity_vector_countdown > 0)
			--gravity_vector_countdown;
	}
}

const char *face_filename = 0;
int face_rotation = 0;

static const double linear_rotation_unit = 2.0;

unsigned int faces_count;
struct ball_face **faces;

struct ball_face
{
	unsigned int rotations;
	cairo_surface_t **c_faces;
};

// Get a random color component in range (255)(R G B)
static double random_color_component()
{
	return 1.0 * (rand() % 200 + 56) / 255;
};

// Initialise the ball face with a given face or random color
struct ball_face *new_ball_face(unsigned int radius, cairo_surface_t *face, int rotation)
{
	struct ball_face *f = malloc(sizeof(struct ball_face));
	if (!f)
		return 0;
	if (face && rotation)
	{
		f->rotations = 2 * M_PI * radius / linear_rotation_unit;
	}
	else
	{
		f->rotations = 1;
	}
	f->c_faces = malloc(sizeof(cairo_surface_t *) * f->rotations);
	if (!f->c_faces)
	{
		free(f);
		return 0;
	}
	for (int i = 0; i < f->rotations; ++i)
	{
		f->c_faces[i] = gdk_window_create_similar_surface(gtk_widget_get_window(window),
														  CAIRO_CONTENT_COLOR_ALPHA,
														  2 * radius, 2 * radius);
		assert(f->c_faces[i]);
		cairo_t *ball_cr = cairo_create(f->c_faces[i]);
		cairo_translate(ball_cr, radius, radius);
		cairo_arc(ball_cr, 0.0, 0.0, radius, 0, 2 * M_PI);
		cairo_clip(ball_cr);

		if (face)
		{
			int face_x_offset = cairo_image_surface_get_width(face) / 2;
			int face_y_offset = cairo_image_surface_get_height(face) / 2;
			cairo_rotate(ball_cr, i * 2 * M_PI / f->rotations);
			cairo_scale(ball_cr, 1.0 * radius / face_x_offset, 1.0 * radius / face_y_offset);
			cairo_set_source_surface(ball_cr, face, -face_x_offset, -face_y_offset);
			cairo_paint(ball_cr);
		}
		else
		{
			cairo_pattern_t *pat;
			pat = cairo_pattern_create_radial(-0.2 * radius, -0.2 * radius, 0.2 * radius,
											  -0.2 * radius, -0.2 * radius, 1.3 * radius);
			double col_r = random_color_component();
			double col_g = random_color_component();
			double col_b = random_color_component();
			cairo_pattern_add_color_stop_rgba(pat, 0, col_r, col_g, col_b, 1);
			cairo_pattern_add_color_stop_rgba(pat, 1, col_r / 3, col_g / 3, col_b / 3, 1);
			cairo_set_source(ball_cr, pat);
			cairo_arc(ball_cr, 0.0, 0.0, radius, 0, 2 * M_PI);
			cairo_fill(ball_cr);
		}
		cairo_surface_flush(f->c_faces[i]);
		cairo_destroy(ball_cr);
	}
	return f;
}

// Initialise the graphics based on faces/surfaces
void init_graphics()
{
	cairo_surface_t *face_surface = 0;

	if (face_filename)
	{
		face_surface = cairo_image_surface_create_from_png(face_filename);
		if (cairo_surface_status(face_surface) != CAIRO_STATUS_SUCCESS)
		{
			cairo_surface_destroy(face_surface);
			face_surface = 0;
			fprintf(stderr, "could not create surface from PNG file %s\n", face_filename);
		}
	}
	if (face_surface)
	{
		faces_count = radius_max + 1 - radius_min;
		faces = malloc(sizeof(struct ball_face *) * faces_count);
		for (unsigned int i = 0; i < faces_count; ++i)
			faces[i] = 0;
		for (struct ball *b = balls; b != balls + n_balls; ++b)
		{
			unsigned int r_idx = b->radius - radius_min;
			if (!faces[r_idx])
				faces[r_idx] = new_ball_face(b->radius, face_surface, face_rotation);
			b->face = faces[r_idx];
		}
		cairo_surface_destroy(face_surface);
	}
	else
	{
		faces_count = n_balls;
		faces = malloc(sizeof(struct ball_face *) * faces_count);
		for (unsigned int i = 0; i < n_balls; ++i)
			balls[i].face = faces[i] = new_ball_face(balls[i].radius, 0, face_rotation);
	}
}

// Free the memory allocated for faces
void destroy_graphics()
{
	if (!faces)
		return;
	for (int i = 0; i < faces_count; ++i)
	{
		if (faces[i])
		{
			if (faces[i]->c_faces)
			{
				for (unsigned int j = 0; j < faces[i]->rotations; ++j)
					cairo_surface_destroy(faces[i]->c_faces[j]);
				free(faces[i]->c_faces);
			}
			free(faces[i]);
		}
	}
	free(faces);
	faces = 0;
	faces_count = 0;
}

void draw_balls_onto_window()
{
	/* clear pixmap */
	GdkWindow *window = gtk_widget_get_window(canvas);
	cairo_region_t *c_region = cairo_region_create();
	GdkDrawingContext *d_context = gdk_window_begin_draw_frame(window, c_region);

	cairo_t *cr = gdk_drawing_context_get_cairo_context(d_context);

	cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, clear_alpha);
	cairo_paint(cr);

	draw_gravity_vector(cr);

	/* draw balls */
	for (const struct ball *b = balls; b != balls + n_balls; ++b)
	{
		cairo_save(cr);
		cairo_translate(cr, b->x - b->radius, b->y - b->radius);
		unsigned int face_id;
		if (b->face->rotations == 1)
			face_id = 0;
		else
		{
			face_id = b->face->rotations * b->angle / (2 * M_PI);
			assert(face_id < b->face->rotations);
			if (face_id >= b->face->rotations)
				face_id %= b->face->rotations;
		}
		cairo_set_source_surface(cr, b->face->c_faces[face_id], 0, 0);
		cairo_paint(cr);
		cairo_restore(cr);
	}
	gdk_window_end_draw_frame(window, d_context);
	cairo_region_destroy(c_region);
}

// Riconfigure the width and height of the window
// at window resizing
gint configure_event(GtkWidget *widget, GdkEventConfigure *event)
{
	if (width == gtk_widget_get_allocated_width(widget) && height == gtk_widget_get_allocated_height(widget))
		return FALSE;

	width = gtk_widget_get_allocated_width(widget);
	height = gtk_widget_get_allocated_height(widget);

	reposition_within_borders();
	return TRUE;
}

gint keyboard_input(GtkWidget *widget, GdkEventKey *event)
{
	if (event->type != GDK_KEY_PRESS)
		return FALSE;
	switch (event->keyval)
	{
	case GDK_KEY_Up:
		g_y -= 10;
		gravity_vector_countdown = gravity_vector_init;
		break;
	case GDK_KEY_Down:
		g_y += 10;
		gravity_vector_countdown = gravity_vector_init;
		break;
	case GDK_KEY_Left:
		g_x -= 10;
		gravity_vector_countdown = gravity_vector_init;
		break;
	case GDK_KEY_Right:
		g_x += 10;
		gravity_vector_countdown = gravity_vector_init;
		break;
	case GDK_KEY_G:
	case GDK_KEY_g:
		gravity_vector_countdown = gravity_vector_init;
		break;
	case GDK_KEY_X:
	case GDK_KEY_x:
		// When drawing the polygon if we press "x" the figure is canceled
		if (!glob_complete)
		{
			glob_complete = 0;
			glob.count = 0;
		}
		break;
	case GDK_KEY_Q:
	case GDK_KEY_q:
		gtk_main_quit();
		break;
	default:
		return FALSE;
	}
	return TRUE;
}

// Destroy the canvas and quit gtk
void destroy_window(void)
{
	gtk_main_quit();
}

void print_usage(const char *progname)
{
	fprintf(stderr,
			"usage: %s [options...]\n"
			"options:\n"
			"\t<width>x<height>\n"
			"\tn=<number of balls>\n"
			"\tfx=<x-force>\n"
			"\tfy=<y-force>\n"
			"\tradius=<min-radius>-<max-radius>\n"
			"\tv=<min-velocity>-<max-velocity>\n"
			"\tdelta=<frame-delta-time> (in seconds)\n"
			"\tface=<filename>\n"
			"\tclear=<clear-alpha>\n"
			"\tstats=<sample-count> :: rendering timing statitstics (0=disabled, default)\n"
			"\tcollisions=<C> :: n=no collisions, s=simple, i=index\n"
			"\t-r :: activate face rotation\n",
			progname);
}

static unsigned int stats_sampling = 0;
static guint64 stats_update_usec = 0;
static unsigned int stats_update_samples = 0;
static guint64 stats_draw_usec = 0;
static unsigned int stats_draw_samples = 0;

gboolean draw_event(GtkWidget *widget, cairo_t *cr, gpointer data)
{
	if (stats_sampling > 0)
	{
		guint64 start = g_get_monotonic_time();
		draw_balls_onto_window();
		stats_draw_usec += g_get_monotonic_time() - start;
		++stats_draw_samples;
	}
	else
	{
		draw_balls_onto_window();
	}
	// Draw the polygon only if is not completed or is a simple one
	// if it is a complex polygon when completed then pop up a message
	if (simple_polygon || !glob_complete)
	{
		do_drawing(cr, widget);
	}
	else if (glob_complete && !simple_polygon)
	{
		cairo_set_source_rgb(cr, 0.5, 0.0, 0.0);

		cairo_select_font_face(cr, "Purisa",
							   CAIRO_FONT_SLANT_NORMAL,
							   CAIRO_FONT_WEIGHT_BOLD);

		cairo_set_font_size(cr, 15);

		cairo_move_to(cr, 20, 30);
		cairo_show_text(cr, "You cannot draw a self intersected polygon!");
	}
	return FALSE;
}
// --------------------------

// Update the velocity of the drawed sphere
static void create_and_add_sphere()
{
	glob.radius = 10;
	// This radius is twice the one from the structure
	// since the cairo drawing patter for the face is different
	unsigned int v_angle_360 = (v_angle_min + rand() % (v_angle_max + 1 - v_angle_min)) % 360;
	glob.v_angle_x = 2 * M_PI * v_angle_360 / 360;
	glob.angle = (rand() % 360) * 2 * M_PI / 360;

	glob.coordx[0] += delta * glob.v_x + delta * delta * g_x / 2.0;
	glob.v_x += delta * g_x;

	glob.coordy[0] += delta * glob.v_y + delta * delta * g_y / 2.0;
	glob.v_y += delta * g_y;

	// polygon_sphere_case_boundaries();

	balls[n_balls - 1].x = glob.coordx[0];
	balls[n_balls - 1].y = glob.coordy[0];
	balls[n_balls - 1].v_x = glob.v_x;
	balls[n_balls - 1].v_y = glob.v_y;
	balls[n_balls - 1].angle = glob.angle;
	balls[n_balls - 1].v_angle = glob.v_angle_x;
	balls[n_balls - 1].radius = glob.radius;
}

// Find the mass of the polygon
static void polygon_mass()
{
	double sum_x = 0;
	double sum_y = 0;
	for (size_t i = 0; i < glob.count; i++)
	{
		int j = i + 1;
		if (j == glob.count)
		{
			j = 0;
		}
		sum_x += glob.coordx[i] * glob.coordy[j];
		sum_y += glob.coordy[i] * glob.coordx[j];
	}
	// Since density = mass/area
	// mass = desnity * area and dfensity = 1
	glob.mass = abs((int)((sum_x - sum_y) / 2));
	// fprintf(stderr, "%d \n", glob.mass);
}

// Calculate the right respond witht the collision between wall and polygon
void collision_wall_response(double x, double y, double nb[])
{
	double vel_vector[] = {glob.v_x, glob.v_y};
	double dis_vector[] = {x - glob.x, y - glob.y};

	double magnitude = sqrt(nb[0] * nb[0] + nb[1] * nb[1]);
	nb[0] /= magnitude;
	nb[1] /= magnitude;

	magnitude = sqrt(dis_vector[0] * dis_vector[0] + dis_vector[1] * dis_vector[1]);
	dis_vector[0] /= magnitude;
	dis_vector[1] /= magnitude;

	double first_term_x = -2 * (dot_product(vel_vector, nb) + glob.v_angle_x * (crossProduct(dis_vector, nb)));
	double second_term_x = 1 / glob.mass + (pow(crossProduct(dis_vector, nb), 2) / glob.moment_inertia_x);

	double first_term_y = -2 * (dot_product(vel_vector, nb) + glob.v_angle_y * (crossProduct(dis_vector, nb)));
	double second_term_y = 1 / glob.mass + (pow(crossProduct(dis_vector, nb), 2) / glob.moment_inertia_y);

	double j_x = first_term_x / second_term_x;
	double j_y = first_term_y / second_term_y;

	double j_vector[] = {j_x, j_y};

	glob.v_x += (1 / glob.mass) * j_x;
	glob.v_y += (1 / glob.mass) * j_y;
	// fprintf(stderr, "%f \n", vel_vector);

	glob.v_angle_x += crossProduct(dis_vector, j_vector) / glob.moment_inertia_x;
	glob.v_angle_y += crossProduct(dis_vector, j_vector) / glob.moment_inertia_y;
	if(glob.v_angle_x > 2 * M_PI){
		glob.v_angle_x = 2*M_PI;
	}
	if(glob.v_angle_x < -2 * M_PI){
		glob.v_angle_x = -2 * M_PI;
	}
	if(glob.v_angle_y > 2 * M_PI){
		glob.v_angle_y = 2*M_PI;
	}
	if(glob.v_angle_y > -2 * M_PI){
		glob.v_angle_y = -2*M_PI;
	}

	// fprintf(stderr, "%f \n", glob.v_angle_x);
}

// Update the polygon state with the gravity and the rotation
static void update_polygon()
{
	// Magari questa condizione è reduntant
	if (glob_complete)
	{

		/* collision check */
		double sin_a = sin(glob.angle);
		double cos_a = cos(glob.angle);

		glob.x += delta * glob.v_x + delta * delta * g_x / 2.0;
		glob.y += delta * glob.v_y + delta * delta * g_y / 2.0;

		for (int i = 0; i < glob.count; ++i)
		{
			double x = glob.x + cos_a * glob.coordx[i] - sin_a * glob.coordy[i];
			double y = glob.y + sin_a * glob.coordx[i] + cos_a * glob.coordy[i];

			double nb[2];
			double bias = 1;
			if ((x < bias && glob.v_x < 0) || (x > width-bias && glob.v_x > 0))
			{
				
				if (x < bias)
				{
					nb[0] = bias;
					nb[1] = -height;
				}
				else
				{
					nb[0] = bias;
					nb[1] = height;
				}

				collision_wall_response(x, y, nb);

				glob.v_x = -glob.v_x;
				break;
			}
			if ((y < bias && glob.v_y < 0) || (y > height-bias && glob.v_y > 0))
			{
				if (y < bias)
				{
					nb[0] = -width;
					nb[1] = bias;
				}
				else
				{
					nb[0] = width;
					nb[1] = bias;
				}

				collision_wall_response(x, y, nb);

				glob.v_y = -glob.v_y;
				break;
			}
		}

		
		glob.v_x += delta * g_x;
		glob.v_y += delta * g_y;
		glob.angle += delta * glob.v_angle_x;
		// Make boundaries on the angular velocity
		
		if (glob.angle > 2 * M_PI)
			glob.angle -= 2 * M_PI;
		else if (glob.angle < 0)
			glob.angle += 2 * M_PI;

		
	}
}
// --------------------------

// Return in which direction the segment turn
double turn(int a, int b, int c)
{
	/* return 1		if a-->b-->c makes a LEFT turn
	 * return -1	if a-->b-->c makes a RIGHT turn
	 * return 0		if a-->b-->c goes straight or makes a 180 degree turn
	 */

	/* vector product ab x bc (vector a-->b cross vector b-->c) */
	double p = (glob.coordx[b] - glob.coordx[a]) * (glob.coordy[c] - glob.coordy[b]) - (glob.coordy[b] - glob.coordy[a]) * (glob.coordx[c] - glob.coordx[b]);
	if (p > 0)
		return 1;
	else if (p < 0)
		return -1;
	else
		return 0;
}

// Chech if the directions are opposite or the same
// If they are the same then the two segments intersect
int intersection(int a, int b, int c, int d)
{
	/* return TRUE iff segment a--b intersects segment c--d
	 */
	return turn(a, b, c) * turn(a, b, d) == -1 && turn(a, c, d) * turn(b, c, d) == -1;
}

// Draw a red line when the intersection occurs
// Note that in this case I use the coordinates
// that are not relative to the center of mass
// since while drawing there is no force applied,
// Thus for an intersection I can use the normal coordinates
void draw_intersections(cairo_t *cr)
{
	for (int i = 0; i < glob.count; ++i)
	{
		int a = i;
		int b = i + 1;
		for (int c = b + 1; c < glob.count; ++c)
		{
			int d = (c + 1 != glob.count) ? c + 1 : 0;
			if (d != a && intersection(a, b, c, d))
			{
				// Intersection found
				simple_polygon = 0;
				cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 0.5);
				cairo_move_to(cr, glob.coordx[a], glob.coordy[a]);
				cairo_line_to(cr, glob.coordx[b], glob.coordy[b]);
				cairo_stroke(cr);
				cairo_move_to(cr, glob.coordx[c], glob.coordy[c]);
				cairo_line_to(cr, glob.coordx[d], glob.coordy[d]);
				cairo_stroke(cr);
			}
		}
	}
}

// Update the state for each object on the canvas
gboolean timeout(gpointer user_data)
{
	if (glob_complete && glob.count > 1)
	{
		update_polygon();
	}
	// Non ho ben capito a cosa serva questa condizione con lo stats_sampling
	if (stats_sampling > 0)
	{
		guint64 start = g_get_monotonic_time();
		update_state();
		stats_update_usec += g_get_monotonic_time() - start;
		if (++stats_update_samples == stats_sampling)
		{
			float uavg = 1.0 * stats_update_usec / stats_update_samples;
			float davg = 1.0 * stats_draw_usec / stats_draw_samples;
			printf("\rupdate = %.0f us, draw = %.0f us, load = %.0f%% (%u update, %u draw)  ",
				   uavg, davg, (uavg + davg) / (10000.0 * delta),
				   stats_update_samples, stats_draw_samples);
			fflush(stdout);
			stats_update_usec = 0;
			stats_update_samples = 0;
			stats_draw_usec = 0;
			stats_draw_samples = 0;
		}
	}
	else
	{
		update_state();
	}
	gtk_widget_queue_draw(canvas);
	return TRUE;
}

// Draw on the screen based on the coordinates choosen
// with the mouse.
// If we choose only one coordinate then it will draw a ball
// on the screen, otherwise it will show with transparency
// the polygon that will be drawn by right clicking on the screen.
// Only once right clicked the figure will be drawn.
// If the user clicks on the "x" while drawing the figure will disappear
static void do_drawing(cairo_t *cr, GtkWidget *widget)
{
	double transparency = 1.0;
	if (!glob_complete)
		transparency = 0.2;

	if (glob.count == 1)
	{
		// If the the user wants to draw a sphere
		// we hide the ghost drawing and let the
		// old implementation run with the new added
		// sphere
		if (glob_complete)
		{
			transparency = 0.0;
		}
		cairo_translate(cr, glob.coordx[0], glob.coordy[0]);
		cairo_arc(cr, 0, 0, 10, 0, 2 * M_PI);

		cairo_pattern_t *pat;
		int radius = 5;
		pat = cairo_pattern_create_radial(-0.2 * radius, -0.2 * radius, 0.2 * radius,
										  -0.2 * radius, -0.2 * radius, 1.3 * radius);
		cairo_pattern_add_color_stop_rgba(pat, 0, 1.0, 1.0, 1.0, transparency);
		cairo_pattern_add_color_stop_rgba(pat, 1, 1.0 / 3, 1.0 / 3, 1.0 / 3, transparency);
		cairo_set_source(cr, pat);

		cairo_fill(cr);
		return;
	}

	double sin_a = sin(glob.angle);
	double cos_a = cos(glob.angle);
	int first_point = 1;
	for (int i = 0; i < glob.count; ++i)
	{
		double x = glob.x + cos_a * glob.coordx[i] - sin_a * glob.coordy[i];
		double y = glob.y + sin_a * glob.coordx[i] + cos_a * glob.coordy[i];
		if (first_point)
		{
			cairo_move_to(cr, x, y);
			first_point = 0;
		}
		else
			cairo_line_to(cr, x, y);
	}
	cairo_close_path(cr);
	cairo_set_source_rgba(cr, 0.1, 0.094, 0.4, transparency);
	cairo_fill_preserve(cr);
	cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, transparency);
	cairo_set_line_width(cr, 2);
	cairo_stroke(cr);

	draw_intersections(cr);

	// draw the gravity point
	for (int i = 0; i < glob.count; ++i)
	{
		double x = glob.x + cos_a * glob.coordx[i] - sin_a * glob.coordy[i];
		double y = glob.y + sin_a * glob.coordx[i] + cos_a * glob.coordy[i];
		cairo_set_source_rgba(cr, 0.0, 1.0, 0.0, 0.5);
		cairo_move_to(cr, x, y);
		cairo_line_to(cr, glob.x, glob.y);
		cairo_stroke(cr);
	}

	// fprintf(stderr, "%f\n", glob.center_of_gravity[0]);
	cairo_translate(cr, glob.x, glob.y);
	cairo_arc(cr, 0, 0, 10, 0, 2 * M_PI);

	cairo_pattern_t *pat;
	int radius = 5;
	pat = cairo_pattern_create_radial(-0.2 * radius, -0.2 * radius, 0.2 * radius,
									  -0.2 * radius, -0.2 * radius, 1.3 * radius);
	cairo_pattern_add_color_stop_rgba(pat, 0, 1.0, 1.0, 1.0, transparency);
	cairo_pattern_add_color_stop_rgba(pat, 1, 1.0 / 3, 1.0 / 3, 1.0 / 3, transparency);
	cairo_set_source(cr, pat);

	cairo_fill(cr);
}

// Calculate the center of mass of the polygon
static void centroidForPoly()
{
	if (glob.count == 2)
	{
		glob.x = glob.coordx[0] + (glob.coordx[1] - glob.coordx[0]) / 2;
		glob.y = glob.coordy[0] + (glob.coordy[1] - glob.coordy[0]) / 2;
	}
	else
	{
		double sum = 0.0;
		double vsum[2];

		for (int i = 0; i < glob.count; i++)
		{
			double cross = glob.coordx[i] * glob.coordy[(i + 1) % glob.count] - glob.coordy[i] * glob.coordx[(i + 1) % glob.count];
			sum += cross;
			vsum[0] = (glob.coordx[i] + glob.coordx[(i + 1) % glob.count]) * cross + vsum[0];
			vsum[1] = (glob.coordy[i] + glob.coordy[(i + 1) % glob.count]) * cross + vsum[1];
		}
		double z = 1.0 / (3.0 * sum);
		glob.x = vsum[0] * z;
		glob.y = vsum[1] * z;
	}
}

// Connect the click event
// When the user clicks on the screen it starts drawing
// If after one click the user right click on the screen
// it will draw a sphere, otherwise a polygon based on the
// points the user has clicked
static gboolean clicked(GtkWidget *widget, GdkEventButton *event,
						gpointer user_data)
{
	if (event->button == 1)
	{
		//  If the user clicks in another place after drawing
		// we reset the drawing
		if (glob_complete)
		{
			glob_complete = 0;
			simple_polygon = 1;
			if (glob.count == 1)
			{
				// Reallocate the memory for the erased sphere
				n_balls -= 1;
				balls = realloc(balls, sizeof(struct ball) * (n_balls));
				assert(balls);

				c_index = realloc(c_index, sizeof(struct bt_node) * (n_balls));
				assert(c_index);
			}
			glob.x = 0.0;
			glob.y = 0.0;
			glob.count = 0;
			glob.angle = 0.0;
		}
		if (glob.count < 100)
		{
			glob.coordx[glob.count] = event->x;
			glob.coordy[glob.count] = event->y;
			glob.count++;
		}
	}
	else if (event->button == 3)
	{

		glob_complete = 1;
		// Add random velocity
		glob.v_x = v_min + rand() % (v_max + 1 - v_min);
		glob.v_y = v_min + rand() % (v_max + 1 - v_min);
		glob.v_angle_x = (v_min + rand() % (v_max + 1 - v_min)) / 10;
		glob.v_angle_y = glob.v_angle_x;

		if (glob.count == 1)
		{
			// Reallocate the memory for the added sphere
			n_balls += 1;

			balls = realloc(balls, sizeof(struct ball) * (n_balls));
			assert(balls);

			c_index = realloc(c_index, sizeof(struct bt_node) * (n_balls));
			assert(c_index);

			// Bisogna fare anche il realloc delle faces.
			// Per ora non copro la condizione di un filename

			create_and_add_sphere();

			// Here we reallocate the memory for the faces
			// and add the info only to the last sphere added to the screen
			cairo_surface_t *face_surface = 0;
			if (face_surface)
			{
				faces_count = radius_max + 1 - radius_min;
				faces = malloc(sizeof(struct ball_face *) * faces_count);
				for (unsigned int i = 0; i < faces_count; ++i)
					faces[i] = 0;

				unsigned int r_idx = balls[-1].radius - radius_min;
				if (!faces[r_idx])
					faces[r_idx] = new_ball_face(balls[-1].radius, face_surface, face_rotation);
				balls[-1].face = faces[r_idx];
				cairo_surface_destroy(face_surface);
			}
			else
			{
				faces_count = n_balls;
				faces = realloc(faces, sizeof(struct ball_face *) * faces_count);
				balls[n_balls - 1].face = faces[n_balls - 1] = new_ball_face(balls[n_balls - 1].radius, 0, face_rotation);
			}
		}
		else
		{
			// Calculate center of gravity
			centroidForPoly();
			// calculate the mass of the polygon
			polygon_mass();
			// calculate the moment of inertia of the polygon
			moment_of_inertia();
			for (int i = 0; i < glob.count; ++i)
			{
				glob.coordx[i] -= glob.x;
				glob.coordy[i] -= glob.y;
			}
		}
		gtk_widget_queue_draw(widget);
	}

	return TRUE;
}

int main(int argc, const char *argv[])
{
	int w = DEFAULT_WIDTH;
	int h = DEFAULT_HEIGHT;
	// Various additional parameters for the running
	// Change window size : ./balls 200x120
	// Change number of elements : ./balls 20
	// Change velocity on x : ./balls fx=120
	// Change velocity on y : ./balls fy=120
	// Change radius range : ./balls radius=10-50 (from 10 to 50)
	// Change velocity range : ./balls v=10-100 (from 10 to 100)
	// DELTA?
	// Change face of the object : ./balls face=<filename>
	// CLEAR?
	// STATS?
	// Change collision type : ./balls collisions=[i = with index (collision with elements),
	// 											   n = no collisions between objects,
	// 											   s = simple collision (collision between objects)]
	for (int i = 1; i < argc; ++i)
	{
		if (sscanf(argv[i], "%dx%d", &w, &h) == 2)
			continue;
		if (sscanf(argv[i], "n=%u", &n_balls) == 1)
			continue;
		if (sscanf(argv[i], "fx=%lf", &g_x) == 1)
			continue;
		if (sscanf(argv[i], "fy=%lf", &g_y) == 1)
			continue;
		if (sscanf(argv[i], "radius=%u-%u", &radius_min, &radius_max) == 2)
			continue;
		if (sscanf(argv[i], "v=%u-%u", &v_min, &v_max) == 2)
			continue;
		if (sscanf(argv[i], "delta=%lf", &delta) == 1)
			continue;
		if (strncmp(argv[i], "face=", 5) == 0)
		{
			face_filename = argv[i] + 5;
			continue;
		}
		if (sscanf(argv[i], "clear=%lf", &clear_alpha) == 1)
			continue;
		if (sscanf(argv[i], "stats=%u", &stats_sampling) == 1)
			continue;
		char collisions;
		if (sscanf(argv[i], "collisions=%c", &collisions) == 1)
		{
			switch (collisions)
			{
			case 'i':
			case 'I':
				check_collisions = check_collisions_with_index;
				continue;
			case '0':
			case 'N':
			case 'n':
				check_collisions = 0;
				continue;
			case 's':
			case 'S':
				check_collisions = check_collisions_simple;
				continue;
			}
		}
		if (strcmp(argv[i], "-r") == 0)
		{
			face_rotation = 1;
			continue;
		}
		print_usage(argv[0]);
		return 1;
	}

	balls = malloc(sizeof(struct ball) * n_balls);
	assert(balls);

	assert(c_index_init());

	balls_init_state();

	// ---------------------------------------------
	glob.count = 0;

	// ---------------------------------------------

	gtk_init(0, 0);

	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_default_size(GTK_WINDOW(window), width, height);
	gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
	gtk_window_set_title(GTK_WINDOW(window), "Game");

	g_signal_connect(window, "destroy", G_CALLBACK(destroy_window), NULL);
	g_signal_connect(G_OBJECT(window), "delete-event", G_CALLBACK(destroy_window), NULL);
	g_signal_connect(window, "key-press-event", G_CALLBACK(keyboard_input), NULL);
	//
	g_signal_connect(window, "button-press-event", G_CALLBACK(clicked), NULL);
	//
	gtk_widget_set_events(window, GDK_EXPOSURE_MASK | GDK_BUTTON_PRESS_MASK | GDK_KEY_PRESS_MASK);

	canvas = gtk_drawing_area_new();

	g_signal_connect(G_OBJECT(canvas), "configure-event", G_CALLBACK(configure_event), NULL);
	g_signal_connect(G_OBJECT(canvas), "draw", G_CALLBACK(draw_event), NULL);
	gtk_container_add(GTK_CONTAINER(window), canvas);

	g_timeout_add(delta * 1000, timeout, canvas);

	gtk_widget_show_all(window);

	init_graphics();

	gtk_main();

	if (stats_sampling > 0)
		printf("\n");

	destroy_graphics();
	c_index_destroy();
	free(balls);

	return 0;
}
