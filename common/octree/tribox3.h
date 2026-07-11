#ifndef _TRIBOX3_H
#define _TRIBOX3_H 1

// Deprecated: use TriBox3.hxx (TriBoxOverlap) instead of linking tribox3.c.

#ifdef __cplusplus
extern "C" {
#endif

  extern int triBoxOverlap(float boxcenter[3],float boxhalfsize[3],float triverts[3][3]);
  
#ifdef __cplusplus
}
#endif

#endif // _TRIBOX3_H
