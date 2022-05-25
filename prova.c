#include <math.h>

#include <gtk/gtk.h>

/* #define CHECK_BEFORE_ADDING 1 */

typedef struct
{
    double x;
    double y;
} point;

/* very primitive realization of a polygon */
unsigned P_size = 0;
point P[100];

int add_polygon_point(double x, double y);

gboolean draw_frame(GtkWidget *widget, cairo_t *cr, gpointer data);

gint keyboard_input(GtkWidget *widget, GdkEventKey *event)
{
    if (event->type != GDK_KEY_PRESS)
        return FALSE;
    switch (event->keyval)
    {
    case GDK_KEY_C:
    case GDK_KEY_c:
        P_size = 0;
        gtk_widget_queue_draw(widget);
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

gint configure_event(GtkWidget *widget, GdkEventConfigure *event)
{
    return TRUE;
}

void destroy_window(void)
{
    gtk_main_quit();
}

gboolean mouse_button(GtkWidget *widget, GdkEvent *event, gpointer user_data)
{
    if (event->type == GDK_BUTTON_PRESS)
    {
        GdkEventButton *e = (GdkEventButton *)event;
        add_polygon_point(e->x, e->y);
        gtk_widget_queue_draw(widget);
    }
    return TRUE;
}

int main(int argc, const char *argv[])
{
    unsigned width = 800;
    unsigned height = 600;

    gtk_init(0, 0);

    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), width, height);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(window), "Statica");

    g_signal_connect(window, "destroy", G_CALLBACK(destroy_window), NULL);
    g_signal_connect(G_OBJECT(window), "delete-event", G_CALLBACK(destroy_window), NULL);
    g_signal_connect(G_OBJECT(window), "key-press-event", G_CALLBACK(keyboard_input), NULL);
    g_signal_connect(G_OBJECT(window), "button-press-event", G_CALLBACK(mouse_button), NULL);
    gtk_widget_set_events(window, GDK_EXPOSURE_MASK | GDK_BUTTON_PRESS_MASK | GDK_KEY_PRESS_MASK);

    GtkWidget *canvas = gtk_drawing_area_new();

    g_signal_connect(G_OBJECT(canvas), "configure-event", G_CALLBACK(configure_event), NULL);
    g_signal_connect(G_OBJECT(canvas), "draw", G_CALLBACK(draw_frame), NULL);

    gtk_container_add(GTK_CONTAINER(window), canvas);

    gtk_widget_show_all(window);

    gtk_main();

    return 0;
}

double turn(const point *a, const point *b, const point *c)
{
    /* return 1		if a-->b-->c makes a LEFT turn
     * return -1	if a-->b-->c makes a RIGHT turn
     * return 0		if a-->b-->c goes straight or makes a 180 degree turn
     */

    /* vector product ab x bc (vector a-->b cross vector b-->c) */
    double p = (b->x - a->x) * (c->y - b->y) - (b->y - a->y) * (c->x - b->x);
    if (p > 0)
        return 1;
    else if (p < 0)
        return -1;
    else
        return 0;
}

int intersection(const point *a, const point *b, const point *c, const point *d)
{
    /* return TRUE iff segment a--b intersects segment c--d
     */
    return turn(a, b, c) * turn(a, b, d) == -1 && turn(c, d, a) * turn(c, d, b) == -1;
}

void draw_intersections(cairo_t *cr)
{
    for (const point *a = P; a + 1 != P + P_size; ++a)
    {
        const point *b = a + 1;
        for (const point *c = b + 1; c != P + P_size; ++c)
        {
            const point *d = (c + 1 != P + P_size) ? c + 1 : P;
            if (d != a && intersection(a, b, c, d))
            {
                cairo_set_source_rgba(cr, 0.0, 1.0, 0.0, 0.5);
                cairo_move_to(cr, a->x, a->y);
                cairo_line_to(cr, b->x, b->y);
                cairo_stroke(cr);
                cairo_move_to(cr, c->x, c->y);
                cairo_line_to(cr, d->x, d->y);
                cairo_stroke(cr);
            }
        }
    }
}

gboolean draw_frame(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
    cairo_paint(cr);
    if (P_size > 0)
    {
        cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 1.0);
        cairo_move_to(cr, P[0].x, P[0].y);
        for (unsigned i = 1; i < P_size; ++i)
            cairo_line_to(cr, P[i].x, P[i].y);
        cairo_close_path(cr);
        cairo_stroke(cr);
#if !CHECK_BEFORE_ADDING
        draw_intersections(cr);
#endif
    }
    return TRUE;
}

int add_polygon_point(double x, double y)
{
    if (P_size == 100)
        return 0;
    P[P_size].x = x;
    P[P_size].y = y;
#if CHECK_BEFORE_ADDING
    if (P_size > 2)
    {
        const point *a = P + P_size;
        const point *b = P + (P_size - 1);
        for (unsigned i = 0; i + 2 < P_size; ++i)
            if (intersection(b, a, P + i, P + (i + 1)))
                return 0;
        b = P;
        for (unsigned i = 1; i + 1 < P_size; ++i)
            if (intersection(a, b, P + i, P + (i + 1)))
                return 0;
    }
#endif
    P_size += 1;
    return 1;
}