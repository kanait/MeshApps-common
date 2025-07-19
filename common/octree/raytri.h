#ifndef _RAYTRI_H
#define _RAYTRI_H 1

#ifdef __cplusplus
extern "C" {
#endif

  // raytri.c に定義されている関数を利用
  extern int intersect_triangle(double orig[3], double dir[3],
				double vert0[3], double vert1[3], double vert2[3],
				double *t, double *u, double *v);
  extern int intersect_triangle1(double orig[3], double dir[3],
				double vert0[3], double vert1[3], double vert2[3],
				double *t, double *u, double *v);
  extern int intersect_triangle2(double orig[3], double dir[3],
				double vert0[3], double vert1[3], double vert2[3],
				double *t, double *u, double *v);
  extern int intersect_triangle3(double orig[3], double dir[3],
				double vert0[3], double vert1[3], double vert2[3],
				double *t, double *u, double *v);
  
#ifdef __cplusplus
}
#endif


#endif // _RAYTRI_H
