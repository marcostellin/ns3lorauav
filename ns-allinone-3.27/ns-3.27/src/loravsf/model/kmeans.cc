// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

#define HUGE_VAL 10000000
#include "ns3/kmeans.h"
 
//typedef struct { double x, y; int group; uint32_t id; } point_t, *point;

namespace ns3 {

double randf(double m)
{
	return m * rand() / (RAND_MAX - 1.);
}
 
point gen_xy(int count, double radius)
{
	double ang, r;
	point p, pt = (point) malloc(sizeof(point_t) * count);
 
	/* note: this is not a uniform 2-d distribution */
	for (p = pt + count; p-- > pt;) {
		ang = randf(2 * M_PI);
		r = randf(radius);
		p->x = r * cos(ang);
		p->y = r * sin(ang);
	}
 
	return pt;
}

point gen_pts ()
{
	point pt = (point) malloc(sizeof(point_t)*12);
	point_t p0;
	point_t p1;
	point_t p2;
	point_t p3;
	point_t p4;
	point_t p5;
	point_t p6;
	point_t p7;
	point_t p8;
	point_t p9;
	point_t p10;
	point_t p11;
	
	p0.x = 1284.9, p0.y = 1123.6;
	p1.x = 1276.9, p1.y = 1122.4;
	p2.x = 1284.4, p2.y = 1108.3;
	p3.x = 1298.5, p3.y = 1114.3;
	p4.x = 1267.6, p4.y = 1126.7;
	p5.x = 1302.2, p5.y = 1119.4;
	p6.x = 1287.5, p6.y = 879.6;
	p7.x = 1259.2, p7.y = 882.579;
	p8.x = 1265.3, p8.y = 876.5670;
	p9.x = 1299.0, p9.y = 862.4150;
	p10.x = 1299.8, p10.y = 862.44;
	p11.x = 1303.2, p11.y = 863.224;
	
	pt[0] = p0;
	pt[1] = p1;
	pt[2] = p2;
	pt[3] = p3;
	pt[4] = p4;
	pt[5] = p5;
	pt[6] = p6;
	pt[7] = p7;
	pt[8] = p8;
	pt[9] = p9;
	pt[10] = p10;
	pt[11] = p11;
	
        
        
        return pt;
	
}
 
inline double dist2(point a, point b)
{
	double x = a->x - b->x, y = a->y - b->y;
	return x*x + y*y;
}
 
inline int
nearest(point pt, point cent, int n_cluster, double *d2)
{
	int i;
  int min_i = -1;
	point c;
	double d;
  double min_d = HUGE_VAL;
 
#	define for_n for (c = cent, i = 0; i < n_cluster; i++, c++)
	for_n {
		min_d = HUGE_VAL;
		min_i = pt->group;
		for_n {
			if (min_d > (d = dist2(c, pt))) {
				min_d = d; min_i = i;
			}
		}
	}
	if (d2) *d2 = min_d;
	return min_i;
}
 
void kpp(point pts, int len, point cent, int n_cent)
{
#	define for_len for (j = 0, p = pts; j < len; j++, p++)
	int j;
	int n_cluster;
	double sum, *d = (double *) malloc(sizeof(double) * len);
 
	point p;
	cent[0] = pts[ rand() % len ];
	for (n_cluster = 1; n_cluster < n_cent; n_cluster++) {
		sum = 0;
		for_len {
			nearest(p, cent, n_cluster, d + j);
			sum += d[j];
		}
		sum = randf(sum);
		for_len {
			if ((sum -= d[j]) > 0) continue;
			cent[n_cluster] = pts[j];
			break;
		}
	}
	for_len p->group = nearest(p, cent, n_cluster, 0);
	free(d);
}
 
point lloyd(point pts, int len, int n_cluster)
{
	int i, j;
  int min_i = -1;
	int changed;
 
	point cent = (point) malloc(sizeof(point_t) * n_cluster), p, c;
 
	/* assign init grouping randomly */
	//for_len p->group = j % n_cluster;
 
	/* or call k++ init */
	kpp(pts, len, cent, n_cluster);
 
	do {
		/* group element for centroids are used as counters */
		for_n { c->group = 0; c->x = c->y = 0; }
		for_len {
			c = cent + p->group;
			c->group++;
			c->x += p->x; c->y += p->y;
		}
		for_n { c->x /= c->group; c->y /= c->group; }
 
		changed = 0;
		/* find closest centroid of each point */
		for_len {
			min_i = nearest(p, cent, n_cluster, 0);
			if (min_i != p->group) {
				changed++;
				p->group = min_i;
			}
		}
	} while (changed > (len >> 10)); /* stop when 99.9% of points are good */
 
	for_n { c->group = i; }
 
	return cent;
}
 
void print_points (point pts, int len)
{
	for (int i = 0; i < len; i++)
	{
		point_t pt = pts[i];
		printf("x=%f, y=%f, group=%d \n", pt.x, pt.y, pt.group);
	}
}

}
 
// #define PTS 12
// #define K 3
// int main()
// {
// 	int i;
// 	point v = gen_pts();
// 	point c = lloyd(v, PTS, K);
// 	//print_eps(v, PTS, c, K);
// 	print_points (v, PTS);
// 	// free(v); free(c);
// 	return 0;
// }
