#ifndef KMEANS_H
#define KMEANS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

namespace ns3 {

typedef struct { double x, y; int group; uint32_t id; } point_t, *point;
 
double randf(double m);
 
point gen_xy(int count, double radius);

point gen_pts ();
 
double dist2(point a, point b);
 
int
nearest(point pt, point cent, int n_cluster, double *d2);
 
void kpp(point pts, int len, point cent, int n_cent);
 
point lloyd(point pts, int len, int n_cluster);
 
void print_points (point pts, int len);

}
#endif
 
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
