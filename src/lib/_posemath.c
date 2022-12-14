/*
   _posemath.c

   C definitions for pose math library data types and
   manipulation functions.

   Modification history:
	 6-Apr-2005  DRE (from DRDC) Added quaternion conjugate, quaternion addition, and quaternion interpolation
	 									quatquatlerp (linear interpolation)	quatquatslerp (spherical linear interpolation) quatquatsquad (spherical cubic interpolation)					
   27-Feb-2004 WPS replace use of atan2 function with my_atan2 conditionally 
   to avoid possible undefined symbol error for rtlinux/rtai.
   31-Jan-2002 WPS added CIRCLE_FUZZ stuff to pmCircleInit
   18-Dec-2001  FMP added streamlined defs of sincos here, removing need
   for separate sincos.c,h
   21-Jun-2001  WPS put in additional paranoid checks for null pointers.
    4-Jan-2001 WPS added Bert Eding/Dirk Maij fixes to pmCircleInit.
   27-Nov-2000 WPS fixed pmRpyRotConvert and pmRotRpyConvert which previously 
   didn't  correctly check the return values of pmRotQuatConvert, pmRpyQuatConvert, pmQuatRpyConvert, pmQuatRotConvert.
   16-Nov-2000 WPS added unit functions. (renaming pmCartNorm ->pmCartUnit)
   16-Nov-2000 WPS modified pmInitLine and pmLinePoint so that the rotations
   would get included.
    6-Apr-2000 WPS implemented pmCylCartConvert, pmCylSphConvert
    4-Feb-2000 WPS modified pmCirclePoint to avoid div by zero when PM_DEBUG not defined.
   27-Jan-1999  FMP fixed bug in pmCircleInit for domain overflow in acos()
   10-Nov-1998  FMP inited r1, r2 vars in pmLineInit()
   15-Jan-1998  FMP added pmHomInv()
   5-Jan-1998  FMP added pmAxisAngleQuatConvert(), pmQuatAxisAngleMult()
   10-Oct-1997  FMP chickened out and checked for divide-by-0 always,
   not just if PM_DEBUG is defined. Set results to DBL_MAX where appropriate.
   17-Aug-1997  FMP removed include mathprnt.h-- not needed here
   11-Jul-1997  FMP changed names from PM_CARTESIAN, for example, to
   PmCartesian, to get rid of name conflicts with mixed C/C++ apps
   19-Jun-1997  FMP added pmLine, pmCircle stuff
   18-Jun-1997  FMP added pmCartPlaneProj()
   14-Apr-1997  FMP created from C parts of posemath.c
*/
/* $Id: _posemath.c,v 1.3 2005/06/10 03:38:43 dave Exp $ */
#include <math.h>
#include <float.h>              /* DBL_MAX */

#if defined(PM_PRINT_ERROR) && defined(rtai)
#undef PM_PRINT_ERROR
#endif

#if defined(PM_DEBUG) && defined(rtai)
#undef PM_DEBUG
#endif

#ifdef PM_PRINT_ERROR
#define PM_DEBUG                /* have to have debug with printing */
#include <stdio.h>
#include <stdarg.h>
#endif
#include "posemath.h"

#include "sincos.h"

#ifndef __unused_parameter__
#ifdef __GNUC__
#if (__GNUC__ >= 3 ) && !defined(MS_WINDOWS_API)
#define __unused_parameter__ __attribute__ ((unused))
#else
#define __unused_parameter__
#endif
#else
#define __unused_parameter__
#endif
#endif

#ifdef USE_MY_ATAN2

static inline double my_atan2(double y, double x)
{
  
  if(y == 0)
    {
      return (x>=0?0:PM_PI);
    }
  else if(x == 0)
    {
      return (y>=0?PM_PI_2:-PM_PI_2);
    }
  else if(x > 0 )
    {
      return(atan(y/x));
    }
  else if(y > 0)
    {
      return(PM_PI - atan(y/-x));
    }
  else 
    {
      return(-PM_PI + atan(-y/-x));
    }
}
#else
#define my_atan2 atan2
#endif
      
/* global error number */
int pmErrno = 0;

#ifdef PM_PRINT_ERROR

void pmPrintError(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);
}

/* error printing function */
void pmPerror(const char *s)
{
  char *pmErrnoString;

  switch (pmErrno)
  {
  case 0:
    /* no error */
    return;

  case PM_ERR:
    pmErrnoString = "unspecified error";
    break;

  case PM_IMPL_ERR:
    pmErrnoString = "not implemented";
    break;

  case PM_NORM_ERR:
    pmErrnoString = "expected normalized value";
    break;

  case PM_DIV_ERR:
    pmErrnoString = "divide by zero";
    break;

  default:
    pmErrnoString = "unassigned error";
    break;
  }

  if (s != 0 &&
      s[0] != 0)
  {
    fprintf(stderr, "%s: %s\n", s, pmErrnoString);
  }
  else
  {
    fprintf(stderr, "%s\n", pmErrnoString);
  }
}

#endif /* PM_PRINT_ERROR */

/* fuzz checker */
#define IS_FUZZ(a,fuzz) (fabs(a) < (fuzz) ? 1 : 0)

/* Pose Math Basis Functions */

/* Scalar functions */

double pmSqrt(double x)
{
  if (x > 0.0)
  {
    return sqrt(x);
  }

  if (x > SQRT_FUZZ)
  {
    return 0.0;
  }

#ifdef PM_PRINT_ERROR
  pmPrintError("sqrt of large negative number\n");
#endif

  return 0.0;
}

/* Translation rep conversion functions */

int pmCartSphConvert(PmCartesian v, PmSpherical * s)
{
  double _r;

  s->theta = my_atan2(v.y, v.x);
  s->r = sqrt(pmSq(v.x) + pmSq(v.y) + pmSq(v.z));
  _r = sqrt(pmSq(v.x) + pmSq(v.y));
  s->phi = my_atan2(_r, v.z);

  return pmErrno = 0;
}

int pmCartCylConvert(PmCartesian v, PmCylindrical * c)
{
  c->theta = my_atan2(v.y, v.x);
  c->r = sqrt(pmSq(v.x) + pmSq(v.y));
  c->z = v.z;

  return pmErrno = 0;
}

int pmSphCartConvert(PmSpherical s, PmCartesian * v)
{
  double _r;

  _r = s.r * sin(s.phi);
  v->z = s.r * cos(s.phi);
  v->x = _r * cos(s.theta);
  v->y = _r * sin(s.theta);

  return pmErrno = 0;
}

int pmSphCylConvert(PmSpherical s, PmCylindrical * c)
{
  c->theta = s.theta;
  c->r = s.r*cos(s.phi);
  c->z = s.r*sin(s.phi);
  return pmErrno = 0;
}

int pmCylCartConvert(PmCylindrical c, PmCartesian * v)
{
  v->x = c.r * cos(c.theta);
  v->y = c.r * sin(c.theta);
  v->z = c.z;
  return pmErrno = 0;
}

int pmCylSphConvert(PmCylindrical c, PmSpherical * s)
{
  s->theta = c.theta;
  s->r = sqrt(pmSq(c.r) + pmSq(c.z));
  s->phi = my_atan2(c.z,c.r);
  return pmErrno = 0;
}

/* Rotation rep conversion functions */

int pmAxisAngleQuatConvert(PmAxis axis, double a, PmQuaternion *q)
{
  double sh;

  a *= 0.5;
  sincos(a, &sh, &(q->s));

  switch (axis)
    {
    case PM_X:
      q->x = sh;
      q->y = 0.0;
      q->z = 0.0;
      break;

    case PM_Y:
      q->x = 0.0;
      q->y = sh;
      q->z = 0.0;
      break;

    case PM_Z:
      q->x = 0.0;
      q->y = 0.0;
      q->z = sh;
      break;

    default:
#ifdef PM_PRINT_ERROR
      pmPrintError("error: bad axis in pmAxisAngleQuatConvert\n");
#endif
      return -1;
      /* break; */
    }

  if( q->s < 0.0 )
    {
      q->s *= -1.0;
      q->x *= -1.0;
      q->y *= -1.0;
      q->z *= -1.0;
    }

  return 0;
}

int pmRotQuatConvert(PmRotationVector r, PmQuaternion * q)
{
  double sh;

#ifdef PM_DEBUG
  /* make sure r is normalized */
  if (0 != pmRotNorm(r, &r))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("error: pmRotQuatConvert rotation vector not normalized\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  if (pmClose(r.s, 0.0, QS_FUZZ))
  {
    q->s = 1.0;
    q->x = q->y = q->z = 0.0;

    return pmErrno = 0;
  }

  sincos(r.s / 2.0, &sh, &(q->s));

  if (q->s >= 0.0)
  {
    q->x = r.x * sh;
    q->y = r.y * sh;
    q->z = r.z * sh;
  }
  else
  {
    q->s *= -1;
    q->x = -r.x * sh;
    q->y = -r.y * sh;
    q->z = -r.z * sh;
  }

  return pmErrno = 0;
}

int pmRotMatConvert(PmRotationVector r, PmRotationMatrix * m)
{
  double s, c, omc;

#ifdef PM_DEBUG
  if (! pmRotIsNorm(r))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad vector in pmRotMatConvert\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  sincos(r.s, &s, &c);

  /* from space book */
  m->x.x = c + pmSq(r.x) * (omc = 1 - c);       /* omc = One Minus Cos */
  m->y.x = -r.z * s + r.x * r.y * omc;
  m->z.x = r.y * s + r.x * r.z * omc;

  m->x.y = r.z * s + r.y * r.x * omc;
  m->y.y = c + pmSq(r.y) * omc;
  m->z.y = -r.x * s + r.y * r.z * omc;

  m->x.z = -r.y * s + r.z * r.x * omc;
  m->y.z = r.x * s + r.z * r.y * omc;
  m->z.z = c + pmSq(r.z) * omc;

  return pmErrno = 0;
}

int pmRotZyzConvert(
		    __unused_parameter__ PmRotationVector r, 
		    __unused_parameter__ PmEulerZyz * zyz)
{
#ifdef PM_DEBUG
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmRotZyzConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
#else
  return PM_IMPL_ERR;
#endif
}

int pmRotZyxConvert(PmRotationVector r, PmEulerZyx * zyx)
{
  PmRotationMatrix m;
  int r1, r2;

  r1 = pmRotMatConvert(r, &m);
  r2 = pmMatZyxConvert(m, zyx);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

int pmRotRpyConvert(PmRotationVector r, PmRpy * rpy)
{
  PmQuaternion q;
  int r1,r2;
  r1 = 0;
  r2 = 0;
  q.s = q.x = q.y = q.z = 0.0;

  r1 = pmRotQuatConvert(r,&q);
  r2 = pmQuatRpyConvert(q,rpy);

  return r1 || r2 ?  pmErrno : 0;
}

int pmQuatRotConvert(PmQuaternion q, PmRotationVector * r)
{
  double sh;

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatRotConvert\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif
  if(r == 0)
    {
      return (pmErrno = PM_ERR);
    }

  sh = sqrt(pmSq(q.x) + pmSq(q.y) + pmSq(q.z));

  if (sh > QSIN_FUZZ)
  {
    r->s = 2.0 * my_atan2(sh, q.s);
    r->x = q.x / sh;
    r->y = q.y / sh;
    r->z = q.z / sh;
  }
  else
  {
    r->s = 0.0;
    r->x = 0.0;
    r->y = 0.0;
    r->z = 0.0;
  }

  return pmErrno = 0;
}

int pmQuatMatConvert(PmQuaternion q, PmRotationMatrix * m)
{
#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatMatConvert\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  /* from space book where e1=q.x e2=q.y e3=q.z e4=q.s */
  // transpose of matrix inserted 
  m->x.x = 1 - 2 * (pmSq(q.y) + pmSq(q.z));	// 1 - (2Y^2 + 2Z^2)
  m->y.x = 2 * (q.x * q.y + q.z * q.s);		// corrected 2XY + 2ZS
  m->z.x = 2 * (q.z * q.x - q.y * q.s);		// corrected 2XZ - 2YS

  m->x.y = 2 * (q.x * q.y - q.z * q.s);		// corrected 2XY - 2ZS
  m->y.y = 1 - 2 * (pmSq(q.z) + pmSq(q.x));	// 1 - (2X^2 + 2Z^2)
  m->z.y = 2 * (q.y * q.z + q.x * q.s);		// corrected 2YZ + 2XS

  m->x.z = 2 * (q.z * q.x + q.y * q.s);		// corrected 2XZ + 2YS
  m->y.z = 2 * (q.y * q.z - q.x * q.s);		// corrected 2YZ - 2XS
  m->z.z = 1 - 2 * (pmSq(q.x) + pmSq(q.y));	// 1 - - (2X^2 + 2Y^2)

  return pmErrno = 0;
}

int pmQuatZyzConvert(PmQuaternion q, PmEulerZyz * zyz)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmQuatMatConvert(q, &m);
  r2 = pmMatZyzConvert(m, zyz);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

int pmQuatZyxConvert(PmQuaternion q, PmEulerZyx * zyx)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmQuatMatConvert(q, &m);
  r2 = pmMatZyxConvert(m, zyx);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

int pmQuatRpyConvert(PmQuaternion q, PmRpy * rpy)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmQuatMatConvert(q, &m);
  r2 = pmMatRpyConvert(m, rpy);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}


int pmMatRotConvert(PmRotationMatrix m, PmRotationVector * r)
{
  PmQuaternion q;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmMatQuatConvert(m,&q);
  r2 = pmQuatRotConvert(q, r);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

/* OLD VERSION
int pmMatQuatConvert(PmRotationMatrix m, PmQuaternion * q)
{
  
     from Stephe's "space" book
     e1 = (c32 - c23) / 4*e4
     e2 = (c13 - c31) / 4*e4
     e3 = (c21 - c12) / 4*e4
     e4 = sqrt(1 + c11 + c22 + c33) / 2

     if e4 == 0
     e1 = sqrt(1 + c11 - c33 - c22) / 2
     e2 = sqrt(1 + c22 - c33 - c11) / 2
     e3 = sqrt(1 + c33 - c11 - c22) / 2
     to determine whether to take the positive or negative sqrt value
     since e4 == 0 indicates a 180* rotation then (0 x y z) = (0 -x -y -z).
     Thus some generallities can be used:
     1) find which of e1, e2, or e3 has the largest magnitude and leave it pos.
     2) if e1 is largest then
     if c21 < 0 then take the negative for e2
     if c31 < 0 then take the negative for e3
     3) else if e2 is largest then
     if c21 < 0 then take the negative for e1
     if c32 < 0 then take the negative for e3
     4) else if e3 is larget then
     if c31 < 0 then take the negative for e1
     if c32 < 0 then take the negative for e2

     Note: c21 in the space book is m.x.y in this C code
   

  double a;

#ifdef PM_DEBUG
  if (! pmMatIsNorm(m))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad matrix in pmMatQuatConvert\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  q->s = 0.5 * sqrt(1.0 + m.x.x + m.y.y + m.z.z);

  if (fabs(q->s) > QS_FUZZ)
  {
    q->x = (m.y.z - m.z.y) / (a = 4 * q->s);
    q->y = (m.z.x - m.x.z) / a;
    q->z = (m.x.y - m.y.x) / a;
  }
  else
  {
    q->s = 0;
    q->x = sqrt(1.0 + m.x.x - m.y.y - m.z.z) / 2.0;
    q->y = sqrt(1.0 + m.y.y - m.x.x - m.z.z) / 2.0;
    q->z = sqrt(1.0 + m.z.z - m.y.y - m.x.x) / 2.0;

    if (q->x > q->y && q->x > q->z)
    {
      if (m.x.y < 0.0)
      {
        q->y *= -1;
      }
      if (m.x.z < 0.0)
      {
        q->z *= -1;
      }
    }
    else if (q->y > q->z)
    {
      if (m.x.y < 0.0)
      {
        q->x *= -1;
      }
      if (m.y.z < 0.0)
      {
        q->z *= -1;
      }
    }
    else
    {
      if (m.x.z < 0.0)
      {
        q->x *= -1;
      }
      if (m.y.z < 0.0)
      {
        q->y *= -1;
      }
    }
  }

  return pmQuatNorm(*q, q);
}
*/
int pmMatQuatConvert(PmRotationMatrix m, PmQuaternion * q)
{
  /*
     from Stephe's "space" book
     e1 = (c32 - c23) / 4*e4
     e2 = (c13 - c31) / 4*e4
     e3 = (c21 - c12) / 4*e4
     e4 = sqrt(1 + c11 + c22 + c33) / 2

     if e4 == 0
     e1 = sqrt(1 + c11 - c33 - c22) / 2
     e2 = sqrt(1 + c22 - c33 - c11) / 2
     e3 = sqrt(1 + c33 - c11 - c22) / 2
     to determine whether to take the positive or negative sqrt value
     since e4 == 0 indicates a 180* rotation then (0 x y z) = (0 -x -y -z).
     Thus some generallities can be used:
     1) find which of e1, e2, or e3 has the largest magnitude and leave it pos.
     2) if e1 is largest then
     if c21 < 0 then take the negative for e2
     if c31 < 0 then take the negative for e3
     3) else if e2 is largest then
     if c21 < 0 then take the negative for e1
     if c32 < 0 then take the negative for e3
     4) else if e3 is larget then
     if c31 < 0 then take the negative for e1
     if c32 < 0 then take the negative for e2

     Note: c21 in the space book is m.x.y in this C code
     */

  double a;
  double T;
  double S;

#ifdef PM_DEBUG
  if (! pmMatIsNorm(m))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad matrix in pmMatQuatConvert\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif
/* rotation matrix functions */

/*        |  m.x.x   m.y.x   m.z.x  |   */
/*   M =  |  m.x.y   m.y.y   m.z.y  |   */
/*        |  m.x.z   m.y.z   m.z.z  |   */

	// compute trace of matrix
	T = 1 + m.x.x + m.y.y + m.z.z;
	if ( T > 0.000001 )
	{
		S = sqrt(T) * 2;
      		q->x = ( m.z.y - m.y.z ) / S;
		q->y = ( m.x.z - m.z.x ) / S;
      		q->z = ( m.y.x - m.x.y ) / S;
		q->s = 0.25 * S;
	
	}
/*
      ? mat[0]  mat[4] mat[ 8] mat[12] ?
  M = ? mat[1]  mat[5] mat[ 9] mat[13] ?
      ? mat[2]  mat[6] mat[10] mat[14] ?
      ? mat[3]  mat[7] mat[11] mat[15] ?

*/
	else // T <= 0.000001
	{
	    if ( m.x.x > m.y.y && m.x.x > m.z.z )  
	    {	// Column 0: 
        	S  = sqrt( 1.0 + m.x.x - m.y.y - m.z.z ) * 2;
	        q->x = 0.25 * S;
        	q->y = (m.y.x + m.x.y ) / S;
	        q->z = (m.x.z + m.z.x ) / S;
	        q->s = (m.z.y - m.y.z ) / S;
    	    }
	    else if ( m.y.y > m.z.z ) 
	    {	// Column 1: 
        	S  = sqrt( 1.0 + m.y.y - m.x.x - m.z.z ) * 2;
        	q->x = ( m.y.x + m.x.y ) / S;
        	q->y = 0.25 * S;
        	q->z = (m.z.y + m.y.z ) / S;
        	q->s = (m.x.z - m.z.x ) / S;
    	    }
	    else
	    {	// Column 2:
        	S  = sqrt( 1.0 + m.z.z - m.x.x - m.y.y ) * 2;
        	q->x = (m.x.z + m.x.z ) / S;
        	q->y = (m.z.y + m.y.z ) / S;
        	q->z = 0.25 * S;
        	q->s = (m.y.x - m.x.y ) / S;
	    }
     	}// end else


  //return pmQuatNorm(*q, q);
  return 0;
}

int pmMatZyzConvert(PmRotationMatrix m, PmEulerZyz * zyz)
{
  zyz->y = my_atan2(sqrt(pmSq(m.x.z) + pmSq(m.y.z)), m.z.z);

  if (fabs(zyz->y) < ZYZ_Y_FUZZ)
  {
    zyz->z = 0.0;
    zyz->y = 0.0;               /* force Y to 0 */
    zyz->zp = my_atan2(-m.y.x, m.x.x);
  }
  else if (fabs(zyz->y - PM_PI) < ZYZ_Y_FUZZ)
  {
    zyz->z = 0.0;
    zyz->y = PM_PI;             /* force Y to 180 */
    zyz->zp = my_atan2(m.y.x, -m.x.x);
  }
  else
  {
    zyz->z = my_atan2(m.z.y, m.z.x);
    zyz->zp = my_atan2(m.y.z, -m.x.z);
  }

  return pmErrno = 0;
}

int pmMatZyxConvert(PmRotationMatrix m, PmEulerZyx * zyx)
{
  zyx->y = my_atan2(-m.x.z, sqrt(pmSq(m.x.x) + pmSq(m.x.y)));

  if (fabs(zyx->y - PM_PI_2) < ZYX_Y_FUZZ)
  {
    zyx->z = 0.0;
    zyx->y = PM_PI_2;           /* force it */
    zyx->x = my_atan2(m.y.x, m.y.y);
  }
  else if (fabs(zyx->y + PM_PI_2) < ZYX_Y_FUZZ)
  {
    zyx->z = 0.0;
    zyx->y = -PM_PI_2;          /* force it */
    zyx->x = -my_atan2(m.y.z, m.y.y);
  }
  else
  {
    zyx->z = my_atan2(m.x.y, m.x.x);
    zyx->x = my_atan2(m.y.z, m.z.z);
  }

  return pmErrno = 0;
}

int pmMatRpyConvert(PmRotationMatrix m, PmRpy * rpy)
{
  rpy->p = my_atan2(-m.x.z, sqrt(pmSq(m.x.x) + pmSq(m.x.y)));

  if (fabs(rpy->p -PM_PI_2) < RPY_P_FUZZ)
  {
    rpy->r = my_atan2(m.y.x, m.y.y);
    rpy->p =PM_PI_2;            /* force it */
    rpy->y = 0.0;
  }
  else if (fabs(rpy->p +PM_PI_2) < RPY_P_FUZZ)
  {
    rpy->r = -my_atan2(m.y.z, m.y.y);
    rpy->p = -PM_PI_2;          /* force it */
    rpy->y = 0.0;
  }
  else
  {
    rpy->r = my_atan2(m.y.z, m.z.z);
    rpy->y = my_atan2(m.x.y, m.x.x);
  }

  return pmErrno = 0;
}

int pmZyzRotConvert(
		    __unused_parameter__ PmEulerZyz zyz, 
		    __unused_parameter__ PmRotationVector * r)
{
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmZyzRotConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmZyzQuatConvert(PmEulerZyz zyz, PmQuaternion * q)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmZyzMatConvert(zyz, &m);
  r2 = pmMatQuatConvert(m, q);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

int pmZyzMatConvert(PmEulerZyz zyz, PmRotationMatrix * m)
{
  double sa, sb, sg;
  double ca, cb, cg;

  sa = sin(zyz.z);
  sb = sin(zyz.y);
  sg = sin(zyz.zp);

  ca = cos(zyz.z);
  cb = cos(zyz.y);
  cg = cos(zyz.zp);

  m->x.x = ca * cb * cg - sa * sg;
  m->y.x = -ca * cb * sg - sa * cg;
  m->z.x = ca * sb;

  m->x.y = sa * cb * cg + ca * sg;
  m->y.y = -sa * cb * sg + ca * cg;
  m->z.y = sa * sb;

  m->x.z = -sb * cg;
  m->y.z = sb * sg;
  m->z.z = cb;

  return pmErrno = 0;
}

int pmZyzRpyConvert(
		    __unused_parameter__ PmEulerZyz zyz, 
		    __unused_parameter__ PmRpy * rpy)
{
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmZyzRpyConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmZyxRotConvert(PmEulerZyx zyx, PmRotationVector * r)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmZyxMatConvert(zyx, &m);
  r2 = pmMatRotConvert(m, r);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}


int pmZyxQuatConvert(PmEulerZyx zyx, PmQuaternion * q)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmZyxMatConvert(zyx, &m);
  r2 = pmMatQuatConvert(m, q);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

int pmZyxMatConvert(PmEulerZyx zyx, PmRotationMatrix * m)
{
  double sa, sb, sg;
  double ca, cb, cg;

  sa = sin(zyx.z);
  sb = sin(zyx.y);
  sg = sin(zyx.x);

  ca = cos(zyx.z);
  cb = cos(zyx.y);
  cg = cos(zyx.x);

  m->x.x = ca * cb;
  m->y.x = ca * sb * sg - sa * cg;
  m->z.x = ca * sb * cg + sa * sg;

  m->x.y = sa * cb;
  m->y.y = sa * sb * sg + ca * cg;
  m->z.y = sa * sb * cg - ca * sg;

  m->x.z = -sb;
  m->y.z = cb * sg;
  m->z.z = cb * cg;

  return pmErrno = 0;
}

int pmZyxZyzConvert(
		    __unused_parameter__ PmEulerZyx zyx, 
		    __unused_parameter__ PmEulerZyz * zyz)
{
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmZyxZyzConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmZyxRpyConvert(
		    __unused_parameter__ PmEulerZyx zyx, 
		    __unused_parameter__ PmRpy * rpy)
{
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmZyxRpyConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmRpyRotConvert(PmRpy rpy, PmRotationVector * r)
{
  PmQuaternion q;
  int r1,r2;
  r1 = 0;
  r2 = 0;
  q.s = q.x = q.y = q.z = 0.0;
  r->s = r->x = r->y = r->z = 0.0;

  r1 = pmRpyQuatConvert(rpy,&q);
  r2 = pmQuatRotConvert(q,r);

  return r1 || r2 ? pmErrno : 0;
}

int pmRpyQuatConvert(PmRpy rpy, PmQuaternion * q)
{
  PmRotationMatrix m;
  int r1, r2;

  /* FIXME-- need direct equations */
  r1 = pmRpyMatConvert(rpy, &m);
  r2 = pmMatQuatConvert(m, q);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}
// MODIFIED
int pmRpyMatConvert(PmRpy rpy, PmRotationMatrix * m)
{
  double sphi, stheta, spsi;
  double cphi, ctheta, cpsi;

  sphi = sin(rpy.y);		// yaw - alpha = phi
  stheta = sin(rpy.p);	// pitch beta = theta
  spsi = sin(rpy.r);		// roll gamma = psi

  cphi = cos(rpy.y);
  ctheta = cos(rpy.p);
  cpsi = cos(rpy.r);

  m->x.x = cphi * ctheta;
  m->x.y = cphi * stheta * spsi - sphi * cpsi;
  m->x.z = cphi * stheta * cpsi + sphi * spsi;

  m->y.x = sphi * ctheta;
  m->y.y = sphi * stheta * spsi + cphi * cpsi;
  m->y.z = sphi * stheta * cpsi - cphi * spsi;

  m->z.x = -stheta;
  m->z.y = ctheta * spsi;
  m->z.z = ctheta * cpsi;

  return pmErrno = 0;
}

int pmRpyZyzConvert(
		    __unused_parameter__ PmRpy rpy, 
		    __unused_parameter__ PmEulerZyz * zyz)
{
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmRpyZyzConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmRpyZyxConvert(
		    __unused_parameter__ PmRpy rpy, 
		    __unused_parameter__ PmEulerZyx * zyx)
{
#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmRpyZyxConvert not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmPoseHomConvert(PmPose p, PmHomogeneous * h)
{
  int r1;

  h->tran = p.tran;
  r1 = pmQuatMatConvert(p.rot, &h->rot);

  return pmErrno = r1;
}

int pmHomPoseConvert(PmHomogeneous h, PmPose * p)
{
  int r1;

  p->tran = h.tran;
  r1 = pmMatQuatConvert(h.rot, &p->rot);

  return pmErrno = r1;
}

/* PmCartesian functions */

int pmCartCartCompare(PmCartesian v1, PmCartesian v2)
{
  if (fabs(v1.x-v2.x) >= V_FUZZ ||
      fabs(v1.y-v2.y) >= V_FUZZ ||
      fabs(v1.z-v2.z) >= V_FUZZ)
  {
    return 0;
  }

  return 1;
}

int pmCartCartDot(PmCartesian v1, PmCartesian v2, double * d)
{
  *d = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

  return pmErrno = 0;
}

int pmCartCartCross(PmCartesian v1, PmCartesian v2, PmCartesian * vout)
{
  vout->x = v1.y * v2.z - v1.z * v2.y;
  vout->y = v1.z * v2.x - v1.x * v2.z;
  vout->z = v1.x * v2.y - v1.y * v2.x;

  return pmErrno = 0;
}

int pmCartMag(PmCartesian v, double * d)
{
  *d = sqrt(pmSq(v.x) + pmSq(v.y) + pmSq(v.z));

  return pmErrno = 0;
}

int pmCartCartDisp(PmCartesian v1, PmCartesian v2, double *d)
{
  *d = sqrt(pmSq(v2.x - v1.x) + pmSq(v2.y - v1.y) + pmSq(v2.z - v1.z));

  return pmErrno = 0;
}

int pmCartCartAdd(PmCartesian v1, PmCartesian v2, PmCartesian * vout)
{
  vout->x = v1.x + v2.x;
  vout->y = v1.y + v2.y;
  vout->z = v1.z + v2.z;

  return pmErrno = 0;
}

int pmCartCartSub(PmCartesian v1, PmCartesian v2, PmCartesian * vout)
{
  vout->x = v1.x - v2.x;
  vout->y = v1.y - v2.y;
  vout->z = v1.z - v2.z;

  return pmErrno = 0;
}

int pmCartScalMult(PmCartesian v1, double d, PmCartesian * vout)
{
  vout->x = v1.x * d;
  vout->y = v1.y * d;
  vout->z = v1.z * d;

  return pmErrno = 0;
}

static int
pm_double_is_zero(double d)
{
  if(d > -DOUBLE_FUZZ && d < DOUBLE_FUZZ)
    {
      return 1;
    }
  return 0;
}


int pmCartScalDiv(PmCartesian v1, double d, PmCartesian * vout)
{
  if (pm_double_is_zero(d))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Divide by 0 in pmCartScalDiv\n");
#endif
    vout->x = DBL_MAX;
    vout->y = DBL_MAX;
    vout->z = DBL_MAX;

    return pmErrno = PM_DIV_ERR;
  }

  vout->x = v1.x / d;
  vout->y = v1.y / d;
  vout->z = v1.z / d;

  return pmErrno = 0;
}

int pmCartNeg(PmCartesian v1, PmCartesian * vout)
{
  vout->x = -v1.x;
  vout->y = -v1.y;
  vout->z = -v1.z;

  return pmErrno = 0;
}

int pmCartInv(PmCartesian v1, PmCartesian * vout)
{
  double size_sq = pmSq(v1.x) + pmSq(v1.y) + pmSq(v1.z);

  if (pm_double_is_zero(size_sq))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Zero vector in pmCartInv\n");
#endif

    vout->x = DBL_MAX;
    vout->y = DBL_MAX;
    vout->z = DBL_MAX;

    return pmErrno = PM_NORM_ERR;
  }

  vout->x = v1.x / size_sq;
  vout->y = v1.y / size_sq;
  vout->z = v1.z / size_sq;

  return pmErrno = 0;
}

/* This used to be called pmCartNorm. */

int pmCartUnit(PmCartesian v, PmCartesian * vout)
{
  double size = sqrt(pmSq(v.x) + pmSq(v.y) + pmSq(v.z));

  if (pm_double_is_zero(size))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Zero vector in pmCartUnit\n");
#endif

    vout->x = DBL_MAX;
    vout->y = DBL_MAX;
    vout->z = DBL_MAX;

    return pmErrno = PM_NORM_ERR;
  }

  vout->x = v.x / size;
  vout->y = v.y / size;
  vout->z = v.z / size;

  return pmErrno = 0;
}


#if 0
/* This is if 0'd out so we can find all the pmCartNorm calls that should
 * be renamed pmCartUnit. 
 * Later we'll put this back. */

int pmCartNorm(PmCartesian v, PmCartesian * vout)
{

  vout->x = v.x ;
  vout->y = v.y ;
  vout->z = v.z ;

  return pmErrno = 0;
}
#endif

int pmCartIsNorm(PmCartesian v)
{
  return sqrt(pmSq(v.x) + pmSq(v.y) + pmSq(v.z)) - 1.0 <
    UNIT_VEC_FUZZ ? 1 : 0;
}

int pmCartCartProj(PmCartesian v1, PmCartesian v2,
                   PmCartesian * vout)
{
  int r1, r2, r3;
  double d;

  r1 = pmCartUnit(v2, &v2);
  r2 = pmCartCartDot(v1, v2, &d);
  r3 = pmCartScalMult(v2, d, vout);

  return pmErrno = r1 || r2 || r3 ? PM_NORM_ERR : 0;
}

int pmCartPlaneProj(PmCartesian v, PmCartesian normal, PmCartesian * vout)
{
  int r1, r2;
  PmCartesian par;

  r1 = pmCartCartProj(v, normal, &par);
  r2 = pmCartCartSub(v, par, vout);

  return pmErrno = r1 || r2 ? PM_NORM_ERR : 0;
}

/* angle-axis functions */

int pmQuatAxisAngleMult(PmQuaternion q, PmAxis axis, double angle, PmQuaternion *pq)
{
  double sh, ch;

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q))
    {
#ifdef PM_PRINT_ERROR
      pmPrintError("error: non-unit quaternion in pmQuatAxisAngleMult\n");
#endif
      return -1;
    }
#endif

  angle *= 0.5;
  sincos(angle, &sh, &ch);

  switch (axis)
    {
    case PM_X:
      pq->s = ch * q.s - sh * q.x;
      pq->x = ch * q.x + sh * q.s;
      pq->y = ch * q.y + sh * q.z;
      pq->z = ch * q.z - sh * q.y;
      break;

    case PM_Y:
      pq->s = ch * q.s - sh * q.y;
      pq->x = ch * q.x - sh * q.z;
      pq->y = ch * q.y + sh * q.s;
      pq->z = ch * q.z + sh * q.x;
      break;

    case PM_Z:
      pq->s = ch * q.s - sh * q.z;
      pq->x = ch * q.x + sh * q.y;
      pq->y = ch * q.y - sh * q.x;
      pq->z = ch * q.z + sh * q.s;
      break;

    default:
#ifdef PM_PRINT_ERROR
      pmPrintError("error: bad axis in pmQuatAxisAngleMult\n");
#endif
      return -1;
      /* break; */
    }

  if (pq->s < 0.0)
    {
      pq->s *= -1.0;
      pq->x *= -1.0;
      pq->y *= -1.0;
      pq->z *= -1.0;
    }

  return 0;
}

/* PmRotationVector functions */

int pmRotScalMult(PmRotationVector r, double s, PmRotationVector * rout)
{
  rout->s = r.s * s;
  rout->x = r.x;
  rout->y = r.y;
  rout->z = r.z;

  return pmErrno = 0;
}

int pmRotScalDiv(PmRotationVector r, double s, PmRotationVector * rout)
{
  if (pm_double_is_zero(s))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Divide by zero in pmRotScalDiv\n");
#endif

    rout->s = DBL_MAX;
    rout->x = r.x;
    rout->y = r.y;
    rout->z = r.z;

    return pmErrno = PM_NORM_ERR;
  }

  rout->s = r.s / s;
  rout->x = r.x;
  rout->y = r.y;
  rout->z = r.z;

  return pmErrno = 0;
}

int pmRotIsNorm(PmRotationVector r)
{
  if (fabs(r.s) < RS_FUZZ ||
      fabs(sqrt(pmSq(r.x) + pmSq(r.y) + pmSq(r.z))) - 1.0 < UNIT_VEC_FUZZ)
  {
    return 1;
  }

  return 0;
}

int pmRotNorm(PmRotationVector r, PmRotationVector * rout)
{
  double size;

  size = sqrt(pmSq(r.x) + pmSq(r.y) + pmSq(r.z));

  if (fabs(r.s) < RS_FUZZ)
  {
    rout->s = 0.0;
    rout->x = 0.0;
    rout->y = 0.0;
    rout->z = 0.0;

    return pmErrno = 0;
  }

  if (pm_double_is_zero(size))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("error: pmRotNorm size is zero\n");
#endif

    rout->s = 0.0;
    rout->x = 0.0;
    rout->y = 0.0;
    rout->z = 0.0;

    return pmErrno = PM_NORM_ERR;
  }

  rout->s = r.s;
  rout->x = r.x / size;
  rout->y = r.y / size;
  rout->z = r.z / size;

  return pmErrno = 0;
}

/* PmRotationMatrix functions */

int pmMatNorm(PmRotationMatrix m, PmRotationMatrix * mout)
{
  /* FIXME */
  *mout = m;

#ifdef PM_PRINT_ERROR
  pmPrintError("error: pmMatNorm not implemented\n");
#endif
  return pmErrno = PM_IMPL_ERR;
}

int pmMatIsNorm(PmRotationMatrix m)
{
  PmCartesian u;

  pmCartCartCross(m.x, m.y, &u);

  return (pmCartIsNorm(m.x) &&
          pmCartIsNorm(m.y) &&
          pmCartIsNorm(m.z) &&
          pmCartCartCompare(u, m.z));
}

int pmMatInv(PmRotationMatrix m, PmRotationMatrix *mout)
{
  /* inverse of a rotation matrix is the transpose */

  mout->x.x = m.x.x;
  mout->x.y = m.y.x;
  mout->x.z = m.z.x;

  mout->y.x = m.x.y;
  mout->y.y = m.y.y;
  mout->y.z = m.z.y;

  mout->z.x = m.x.z;
  mout->z.y = m.y.z;
  mout->z.z = m.z.z;

  return pmErrno = 0;
}

int pmMatCartMult(PmRotationMatrix m, PmCartesian v, PmCartesian * vout)
{
  vout->x = m.x.x * v.x + m.y.x * v.y + m.z.x * v.z;
  vout->y = m.x.y * v.x + m.y.y * v.y + m.z.y * v.z;
  vout->z = m.x.z * v.x + m.y.z * v.y + m.z.z * v.z;

  return pmErrno = 0;
}

int pmMatMatMult(PmRotationMatrix m1, PmRotationMatrix m2,
                 PmRotationMatrix * mout)
{
  mout->x.x = m1.x.x * m2.x.x + m1.y.x * m2.x.y + m1.z.x * m2.x.z;
  mout->x.y = m1.x.y * m2.x.x + m1.y.y * m2.x.y + m1.z.y * m2.x.z;
  mout->x.z = m1.x.z * m2.x.x + m1.y.z * m2.x.y + m1.z.z * m2.x.z;

  mout->y.x = m1.x.x * m2.y.x + m1.y.x * m2.y.y + m1.z.x * m2.y.z;
  mout->y.y = m1.x.y * m2.y.x + m1.y.y * m2.y.y + m1.z.y * m2.y.z;
  mout->y.z = m1.x.z * m2.y.x + m1.y.z * m2.y.y + m1.z.z * m2.y.z;

  mout->z.x = m1.x.x * m2.z.x + m1.y.x * m2.z.y + m1.z.x * m2.z.z;
  mout->z.y = m1.x.y * m2.z.x + m1.y.y * m2.z.y + m1.z.y * m2.z.z;
  mout->z.z = m1.x.z * m2.z.x + m1.y.z * m2.z.y + m1.z.z * m2.z.z;

  return pmErrno = 0;
}

/* PmQuaternion functions */

int pmQuatQuatAdd(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout)
{
  // check for null pointer, then exit if so
  if(qout == 0)
  {
      return pmErrno = PM_ERR;
  }
  // simple addition of elements for quaternion addition
  qout->s = q1.s + q2.s;
  qout->x = q1.x + q2.x;
  qout->y = q1.y + q2.y;
  qout->z = q1.z + q2.z;

	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", qout->s, qout->x, qout->y, qout->z );	

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q1) ||
      ! pmQuatIsNorm(q2))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatQuatAdd\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  return pmErrno = 0;
}

int pmQuatQuatCompare(PmQuaternion q1, PmQuaternion q2)
{
#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q1) ||
      ! pmQuatIsNorm(q2))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatQuatCompare\n");
#endif
  }
#endif

  if (fabs(q1.s-q2.s) < Q_FUZZ &&
      fabs(q1.x-q2.x) < Q_FUZZ &&
      fabs(q1.y-q2.y) < Q_FUZZ &&
      fabs(q1.z-q2.z) < Q_FUZZ)
  {
    return 1;
  }

  /* note (0, x, y, z) = (0, -x, -y, -z) */
  if (fabs(q1.s) >= QS_FUZZ ||
      fabs(q1.x + q2.x) >= Q_FUZZ ||
      fabs(q1.y + q2.y) >= Q_FUZZ ||
      fabs(q1.z + q2.z) >= Q_FUZZ)
  {
    return 0;
  }

  return 1;
}

int pmQuatQuatDotProduct(PmQuaternion q1, PmQuaternion q2, double * dot)
{
	// compute dot product
	*dot = q1.s*q2.s + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;

	//printf( "dotproduct: %f\n", *dot);

  return pmErrno = 0;
}

int pmQuatQuatLerp(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout, double h)
{

	// fcn returns the linear interpolation between
	// a pair of quaternions q1 and q2. h is the blending factor
	// h should be in the range [0,1]

  PmQuaternion blend_q1; // blended version of q1
	PmQuaternion blend_q2; // blended version of q2	
	PmQuaternion sum_qout; // intermediate summation
	double		absval;									 // absolute value of the summation

  // check for null pointer, then exit if so
  if(qout == 0)
  {
		return pmErrno = PM_ERR;
  }
	// check is blending factor is within [0,1] range
	if (h > 1.0 || h < 0.0)
	{
		return pmErrno = PM_ERR;
	}

	// scalar multiply both by the blending factor
	pmQuatScalMult( q1, (1-h) , &blend_q1); // use 1-h for first
	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", blend_q1.s, blend_q1.x, blend_q1.y, blend_q1.z );
	pmQuatScalMult( q2, (h) , &blend_q2); // use h for second
	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", blend_q2.s, blend_q2.x, blend_q2.y, blend_q2.z );	

	// add blended quaternions together
	pmQuatQuatAdd( blend_q1, blend_q2, &sum_qout);
	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", sum_qout.s, sum_qout.x, sum_qout.y, sum_qout.z );	
	
	// take the norm of intermediate sum
	// Norm(q) = s^2+x^2+y^2+z^2
	absval = sqrt( (sum_qout.s*sum_qout.s) + (sum_qout.x * sum_qout.x) + (sum_qout.y * sum_qout.y) + (sum_qout.z * sum_qout.z) );
	//printf("absval: %f \n", absval );
		
	// store intermediate value adjusted by absolute value
	qout->s = sum_qout.s/absval;
	qout->x = sum_qout.x/absval;
	qout->y = sum_qout.y/absval;	
	qout->z = sum_qout.z/absval;
	
	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", qout->s, qout->x, qout->y, qout->z );
	// does not renormalize quaternions, although probably should
	// uncomment this line to renormalize
	//pmQuatNorm( *qout, qout);
		
  return pmErrno = 0;


}// end pmQuatQuatLerp

int pmQuatQuatSlerp(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout, double h)
{

	// fcn returns the spherical linear interpolation between
	// a pair of unit quaternions q1 and q2. h is the blending factor
	// h should be in the range [0,1]

	register  PmQuaternion temp_q1; 	// intermediate q1
	register  PmQuaternion temp_q2; 	// intermediate q2
	double		angle;									// angle derived from quaternions
	double 		dot;										// dot product of q1q2
	double 		sinfactor;							// sine factor 


  // check for null pointer, then exit if so
  if(qout == 0)
  {
		return pmErrno = PM_ERR;
  }
	// check is blending factor is within [0,1] range?
	if (h > 1.0 || h < 0.0)
	{
		return pmErrno = PM_ERR;
	}
	// find angle from quaternion multiplication
	// multiply first and second quaternion and store  
	pmQuatQuatDotProduct( q1, q2, &dot);
	angle = acos( dot );
	//printf ("dotproduct: %f angle: %f \n", dot, angle);
	
	// sine factor
	sinfactor = sin(angle);
	// compute q1 intermediate
	temp_q1.s = q1.s*sinfactor*(1-h)/sinfactor;
	temp_q1.x = q1.x*sinfactor*(1-h)/sinfactor;
	temp_q1.y = q1.y*sinfactor*(1-h)/sinfactor;
	temp_q1.z = q1.z*sinfactor*(1-h)/sinfactor;		
	
	// compute q2 intermediate
	temp_q2.s = q2.s*sinfactor*(h)/sinfactor;
	temp_q2.x = q2.x*sinfactor*(h)/sinfactor;
	temp_q2.y = q2.y*sinfactor*(h)/sinfactor;
	temp_q2.z = q2.z*sinfactor*(h)/sinfactor;	
	
	// multiply first conjugate and second quaternion 
	pmQuatQuatAdd( temp_q1, temp_q2, qout);

	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", qout->s, qout->x, qout->y, qout->z );	
		
  return pmErrno = 0;


}// end pmQuatQuatSlerp


int pmQuatQuatSquad(PmQuaternion q1, PmQuaternion q2, PmQuaternion q3, PmQuaternion q4, PmQuaternion * qout, double h)
{

	// fcn returns the spherical cubic interpolation between q1 to q2 to q3 to q4
	// for a series of unit quaternions. h is the blending factor
	// h should be in the range [0,1]

  register	PmQuaternion qa; // find spline point within q1 to q2
	register  PmQuaternion qb; // find spline point within q3 to q4

  // check for null pointer, then exit if so
  if(qout == 0)
  {
		return pmErrno = PM_ERR;
  }
	// check is blending factor is within [0,1] range?
	if (h > 1.0 || h < 0.0)
	{
		return pmErrno = PM_ERR;
	}

	// find the intermediate from q1 to q2
	pmQuatQuatSlerp( q1,  q2, &qa,  h);
	
	// find the intermediate from q3 to q4
	pmQuatQuatSlerp( q3,  q4, &qb,  h);
	
	// now compute smoothed spline path using modified blending factor
	pmQuatQuatSlerp( qa,  qb, qout,  2*h*(1-h));
	
		
  return pmErrno = 0;


}// end pmQuatQuatSquad

int pmQuatConjugate(PmQuaternion q1, PmQuaternion * qout)
{

  // fcn returns the conjugate of q1 as qout
  // works for both unit and non-unit quaternions
	
	
  // check for null pointer, then exit if so
  if(qout == 0)
  {
      return pmErrno = PM_ERR;
  }
  // The scalar remains identical and the vector elements have the opposite sign
  qout->s = q1.s;
  qout->x = -1 * q1.x;
  qout->y = -1 * q1.y;
  qout->z = -1 * q1.z;


#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q1) ||
      ! pmQuatIsNorm(q2))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatQuatAdd\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  return pmErrno = 0;
}


int pmQuatMag(PmQuaternion q, double * d)
{
  PmRotationVector r;
  int r1;

  if(0 == d)
    {
      return (pmErrno = PM_ERR);
    }

  r1 = pmQuatRotConvert(q, &r);
  *d = r.s;

  return pmErrno = r1;
}

int pmQuatNorm(PmQuaternion q1, PmQuaternion * qout)
{
  double size = sqrt(pmSq(q1.s) + pmSq(q1.x) + pmSq(q1.y) + pmSq(q1.z));

  if (pm_double_is_zero(size))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatNorm\n");
#endif
    qout->s = 1;
    qout->x = 0;
    qout->y = 0;
    qout->z = 0;

    return pmErrno = PM_NORM_ERR;
  }

  if (q1.s >= 0.0)
  {
    qout->s = q1.s / size;
    qout->x = q1.x / size;
    qout->y = q1.y / size;
    qout->z = q1.z / size;

    return pmErrno = 0;
  }
  else
  {
    qout->s = -q1.s / size;
    qout->x = -q1.x / size;
    qout->y = -q1.y / size;
    qout->z = -q1.z / size;

    return pmErrno = 0;
  }
}

int pmQuatInv(PmQuaternion q1, PmQuaternion * qout)
{
	// fcn returns the inverse only if the quaternion is unitary
	
	// if NULL pointer then return error
  if(qout == 0)
  {
      return pmErrno = PM_ERR;
  }
	// this is in fact the conjugate of the quaternion
	// it assumes that q1 is a unit quaternion
  qout->s = q1.s;
  qout->x = -1 * q1.x;
  qout->y = -1 * q1.y;
  qout->z = -1 * q1.z;

	// the correct form of the inverse is INV(q1)= q1*
	//																				    -------
	//																					  |q1|^2
	// this will need to be changed to accomodate non-unitary quaternions
	

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q1))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatInv\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  return pmErrno = 0;
}

int pmQuatIsNorm(PmQuaternion q1)
{
  return (fabs(pmSq(q1.s) + pmSq(q1.x) + pmSq(q1.y) + pmSq(q1.z) - 1.0) <
          UNIT_QUAT_FUZZ);
}

int pmQuatScalMult(PmQuaternion q, double s, PmQuaternion * qout)
{
  /* This version multiplies the quaternion by a scalar */
	// s is the scalar, q is the input quaternion
	// Does not guaranteee a unit quaternion output
	
	
	qout->s = q.s * s;
	qout->x = q.x * s;
  qout->y = q.y * s;
	qout->z = q.z * s;
	
	
	return pmErrno = 0;
}

int pmQuatScalDiv(PmQuaternion q, double s, PmQuaternion * qout)
{
  /* FIXME-- need a native version; this goes through a rotation vector */
  PmRotationVector r;
  int r1, r2, r3;

  r1 = pmQuatRotConvert(q, &r);
  r2 = pmRotScalDiv(r, s, &r);
  r3 = pmRotQuatConvert(r, qout);

  return pmErrno = (r1 || r2 || r3) ? PM_NORM_ERR : 0;
}

int pmQuatQuatDistance(PmQuaternion q1, PmQuaternion q2, double * result)
{

	// this fcn returns the scalar distance between two quaternions
	// the distance metric is the Euclidean distance metric.
	// the distance is Dist(q1,q2) = min [E(q1,q2), E(q1,-q2) ]
	PmQuaternion q2_neg; 	// negative of q2

	double second_result;	// stores alternate value	
	
	// store the negative of q2
	pmQuatScalMult(q2,  -1, &q2_neg);
	//printf("q.s: %f q.x: %f q.y: %f q.z: %f \n", q2_neg.s, q2_neg.x, q2_neg.y, q2_neg.z );
	
	//compute distance metric #1
	*result = sqrt( (q1.s-q2.s)*(q1.s-q2.s) + (q1.x-q2.x)*(q1.x-q2.x) + (q1.y-q2.y)*(q1.y-q2.y) + (q1.z-q2.z)*(q1.z-q2.z) );
	//printf("result1:%f \n", *result );
	
	// compute distance metric #2 
	second_result = sqrt( (q1.s-q2_neg.s)*(q1.s-q2_neg.s) + (q1.x-q2_neg.x)*(q1.x-q2_neg.x) + (q1.y-q2_neg.y)*(q1.y-q2_neg.y) + (q1.z-q2_neg.z)*(q1.z-q2_neg.z) ) ;
	//printf("result2:%f \n", second_result );
		
	// take the min value
	if ( *result > second_result )
	{
		// store min value in result
		*result = second_result;
	}
	//printf("final result:%f \n", *result );	
	
	return 0;
}// end pmQuatQuatDistance

int pmQuatQuatMult(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout)
{
  if(qout == 0)
  {
      return pmErrno = PM_ERR;
  }

  qout->s = q1.s * q2.s - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;

  if (qout->s >= 0.0)
  {
    qout->x = q1.s * q2.x + q1.x * q2.s + q1.y * q2.z - q1.z * q2.y;
    qout->y = q1.s * q2.y - q1.x * q2.z + q1.y * q2.s + q1.z * q2.x;
    qout->z = q1.s * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.s;
  }
  else
  {
    qout->s *= -1;
    qout->x = - q1.s * q2.x - q1.x * q2.s - q1.y * q2.z + q1.z * q2.y;
    qout->y = - q1.s * q2.y + q1.x * q2.z - q1.y * q2.s - q1.z * q2.x;
    qout->z = - q1.s * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.s;
  }

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q1) ||
      ! pmQuatIsNorm(q2))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatQuatMult\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  return pmErrno = 0;
}

int pmQuatCartMult(PmQuaternion q1, PmCartesian v2, PmCartesian * vout)
{
  PmCartesian c;

  c.x = q1.y * v2.z  -  q1.z * v2.y;
  c.y = q1.z * v2.x  -  q1.x * v2.z;
  c.z = q1.x * v2.y  -  q1.y * v2.x;

  vout->x = v2.x + 2.0 * (q1.s * c.x + q1.y * c.z - q1.z * c.y);
  vout->y = v2.y + 2.0 * (q1.s * c.y + q1.z * c.x - q1.x * c.z);
  vout->z = v2.z + 2.0 * (q1.s * c.z + q1.x * c.y - q1.y * c.x);

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(q1))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmQuatCartMult\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  return pmErrno = 0;
}

/* PmPose functions*/

int pmPosePoseCompare(PmPose p1, PmPose p2)
{
#ifdef PM_DEBUG
  if (! pmQuatIsNorm(p1.rot) ||
      ! pmQuatIsNorm(p2.rot))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmPosePoseCompare\n");
#endif
  }
#endif

  return pmErrno = (pmQuatQuatCompare(p1.rot, p2.rot) &&
          pmCartCartCompare(p1.tran, p2.tran));
}

int pmPoseInv(PmPose p1, PmPose * p2)
{
  int r1, r2;

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(p1.rot))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmPoseInv\n");
#endif
  }
#endif

  r1 = pmQuatInv(p1.rot, &p2->rot);
  r2 = pmQuatCartMult(p2->rot, p1.tran, &p2->tran);

  p2->tran.x *= -1.0;
  p2->tran.y *= -1.0;
  p2->tran.z *= -1.0;

  return pmErrno = (r1 || r2) ? PM_NORM_ERR : 0;
}

int pmPoseCartMult(PmPose p1, PmCartesian v2, PmCartesian * vout)
{
  int r1, r2;

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(p1.rot))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmPoseCartMult\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  r1 = pmQuatCartMult(p1.rot, v2, vout);
  r2 = pmCartCartAdd(p1.tran, *vout, vout);

  return pmErrno = (r1 || r2) ? PM_NORM_ERR : 0;
}

int pmPosePoseMult(PmPose p1, PmPose p2, PmPose * pout)
{
  int r1, r2, r3;

#ifdef PM_DEBUG
  if (! pmQuatIsNorm(p1.rot) ||
      ! pmQuatIsNorm(p2.rot))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad quaternion in pmPosePoseMult\n");
#endif
    return pmErrno = PM_NORM_ERR;
  }
#endif

  r1 = pmQuatCartMult(p1.rot, p2.tran, &pout->tran);
  r2 = pmCartCartAdd(p1.tran, pout->tran, &pout->tran);
  r3 = pmQuatQuatMult(p1.rot, p2.rot, &pout->rot);

  return pmErrno = (r1 || r2 || r3) ? PM_NORM_ERR : 0;
}

/* homogeneous transform functions */

int pmHomInv(PmHomogeneous h1, PmHomogeneous *h2)
{
  int r1, r2;

#ifdef PM_DEBUG
  if (! pmMatIsNorm(h1.rot))
  {
#ifdef PM_PRINT_ERROR
    pmPrintError("Bad rotation matrix in pmHomInv\n");
#endif
  }
#endif

  r1 = pmMatInv(h1.rot, &h2->rot);
  r2 = pmMatCartMult(h2->rot, h1.tran, &h2->tran);

  h2->tran.x *= -1.0;
  h2->tran.y *= -1.0;
  h2->tran.z *= -1.0;

  return pmErrno = (r1 || r2) ? PM_NORM_ERR : 0;
}

/* line functions */

int pmLineInit(PmLine * line, PmPose start, PmPose end)
{
  int r1 = 0, r2 = 0, r3 = 0, r4 = 0, r5 = 0;
  double tmag = 0.0;
  double rmag = 0.0;
  PmQuaternion startQuatInverse;

  if(0 == line)
    {
      return (pmErrno = PM_ERR);
    }

  r3 = pmQuatInv(start.rot,&startQuatInverse);
  if(r3) 
    {
      return r3;
    }

  r4 = pmQuatQuatMult(startQuatInverse,end.rot,&line->qVec);
  if(r4)
    {
      return r4;
    }

  pmQuatMag(line->qVec,&rmag);
  if (rmag > Q_FUZZ) 
    {
      r5 = pmQuatScalMult(line->qVec,1/rmag,&(line->qVec));
      if(r5)
	{
	  return r5;
	}
    }

  line->start = start;
  line->end = end;
  r1 = pmCartCartSub(end.tran, start.tran, &line->uVec);
  if(r1)
    {
      return r1;
    }

  pmCartMag(line->uVec, &tmag);
  if (IS_FUZZ(tmag, CART_FUZZ))
    {
      line->uVec.x = 1.0;
      line->uVec.y = 0.0;
      line->uVec.z = 0.0;
    }
  else
    {
      r2 = pmCartUnit(line->uVec, &line->uVec);
    }
  line->tmag = tmag;
  line->rmag = rmag;
  line->tmag_zero = (line->tmag <= CART_FUZZ);
  line->rmag_zero = (line->rmag <= Q_FUZZ);

  /* return PM_NORM_ERR if uVec has been set to 1, 0, 0 */
  return pmErrno = (r1 || r2 || r3 || r4 || r5) ? PM_NORM_ERR : 0;
}

int pmLinePoint(PmLine * line, double len, PmPose * point)
{
  int r1=0, r2 =0,r3 = 0,r4=0;

  if(line->tmag_zero)
    {
      point->tran = line->end.tran;
    }      
  else
    {
      /* return start + len * uVec */
      r1 = pmCartScalMult(line->uVec, len, &point->tran);
      r2 = pmCartCartAdd(line->start.tran, point->tran, &point->tran);
    }

  if(line->rmag_zero)
    {
      point->rot = line->end.rot;
    }
  else
    {
      if(line->tmag_zero)
	{
	  r3 = pmQuatScalMult(line->qVec, len, &point->rot);
	}
      else
	{
	  r3 = pmQuatScalMult(line->qVec, len*line->rmag/line->tmag, &point->rot);
	}
      r4 = pmQuatQuatMult(line->start.rot,point->rot,&point->rot);
    }

  return pmErrno = (r1 || r2 || r3 || r4) ? PM_NORM_ERR : 0;
}

/* circle functions */

/*
  pmCircleInit() takes the defining parameters of a generalized circle
  and sticks them in the structure. It also computes the radius and vectors
  in the plane that are useful for other functions and that don't need
  to be recomputed every time.

  Note that the end can be placed arbitrarily, resulting in a combination of
  spiral and helical motion. There is an overconstraint between the start,
  center, and normal vector: the center vector and start vector are assumed
  to be in the plane defined by the normal vector. If this is not true, then
  it will be made true by moving the center vector onto the plane.
  */
int pmCircleInit(PmCircle * circle,
                 PmPose start, PmPose end,
                 PmCartesian center, PmCartesian normal,
                 int turn)
{
  double dot;
  PmCartesian rEnd;
  PmCartesian v;
  double d;
  int r1;

#ifdef PM_DEBUG
  if (0 == circle)
    {
#ifdef PM_PRINT_ERROR
      pmPrintError("error: pmCircleInit cirle pointer is null\n");
#endif
      return pmErrno = PM_ERR;
    }
#endif

  /* adjust center */
  pmCartCartSub(start.tran, center, &v);
  r1 = pmCartCartProj(v, normal, &v);
  if (PM_NORM_ERR == r1)
    {
      /* bad normal vector-- abort */
#ifdef PM_PRINT_ERROR
       pmPrintError("error: pmCircleInit normal vector is 0\n");
#endif
      return -1;
    }
  pmCartCartAdd(v, center, &circle->center);

  /* normalize and redirect normal vector based on turns. If turn is
     less than 0, point normal vector in other direction and make
     turn positive, -1 -> 0, -2 -> 1, etc. */
  pmCartUnit(normal, &circle->normal);
  if (turn < 0)
    {
      turn = -1 - turn;
      pmCartScalMult(circle->normal, -1.0, &circle->normal);
    }

  /* radius */
  pmCartCartDisp(start.tran, circle->center, &circle->radius);

  /* vector in plane of circle from center to start, magnitude radius */
  pmCartCartSub(start.tran, circle->center, &circle->rTan);
  /* vector in plane of circle perpendicular to rTan, magnitude radius */
  pmCartCartCross(circle->normal, circle->rTan, &circle->rPerp);

  /* do rHelix, rEnd */
  pmCartCartSub(end.tran, circle->center, &circle->rHelix);
  pmCartPlaneProj(circle->rHelix, circle->normal, &rEnd);
  pmCartMag(rEnd, &circle->spiral);
  circle->spiral -= circle->radius;
  pmCartCartSub(circle->rHelix, rEnd, &circle->rHelix);
  pmCartUnit(rEnd, &rEnd);
  pmCartScalMult(rEnd, circle->radius, &rEnd);

 /* Patch for error spiral end same as spiral center */
  pmCartMag(rEnd, &d);
  if(pm_double_is_zero(d))
    {
      pmCartScalMult(circle->normal, DOUBLE_FUZZ, &v);
      pmCartCartAdd(rEnd, v, &rEnd);
    }
  /* end patch 03-mar-1999 Dirk Maij*/
  

  /* angle */
  pmCartCartDot(circle->rTan, rEnd, &dot);
  dot = dot / (circle->radius * circle->radius);
  if (dot > 1.0)
    {
      circle->angle = 0.0;
    }
  else if (dot < -1.0)
    {
      circle->angle = PM_PI;
    }
  else
    {
      circle->angle = acos(dot);
    }
  /* now angle is in range 0..PI . Check if cross is antiparallel to
     normal. If so, true angle is between PI..2PI. Need to subtract
     from 2PI. */
  pmCartCartCross(circle->rTan, rEnd, &v);
  pmCartCartDot(v, circle->normal, &d);
  if (d < 0.0)
    {
      circle->angle = PM_2_PI - circle->angle;
    }

  if (circle->angle > -(CIRCLE_FUZZ) && circle->angle < (CIRCLE_FUZZ) )
  {
     circle->angle = PM_2_PI; 
  }

  /* now add more angle for multi turns */
  if (turn > 0)
    {
      circle->angle += turn * 2.0 * PM_PI;
    }
  
#if 0
  printf("\n\n");
  printf("pmCircleInit:\n");
  printf(" \t start  : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 start.tran.x,start.tran.y,start.tran.z);
  printf(" \t end    : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 end.tran.x,end.tran.y,end.tran.z);
  printf(" \t center : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 center.x,center.y,center.z);
  printf(" \t normal : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 normal.x,normal.y,normal.z);
  printf(" \t rEnd   : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 rEnd.x,rEnd.y,rEnd.z);
  printf(" \t turn=%d\n", turn);
  printf(" \t dot=%9.9f\n", dot);
  printf(" \t d=%9.9f\n", d);
  printf(" \t circle  \t{angle=%9.9f, radius=%9.9f, spiral=%9.9f}\n",
	 circle->angle,circle->radius, circle->spiral);
  printf(" \t circle->normal : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 circle->normal.x,circle->normal.y,circle->normal.z);
  printf(" \t circle->center : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 circle->center.x,circle->center.y,circle->center.z);
  printf(" \t circle->rTan : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 circle->rTan.x,circle->rTan.y,circle->rTan.z);
  printf(" \t circle->rPerp : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 circle->rPerp.x,circle->rPerp.y,circle->rPerp.z);
  printf(" \t circle->rHelix : \t{x=%9.9f, y=%9.9f, z=%9.9f}\n",
	 circle->rHelix.x,circle->rHelix.y,circle->rHelix.z);
  printf("\n\n");
#endif 


  return pmErrno = 0;
}

/*
  pmCirclePoint() returns the vector to the point at the given angle along
  the circle. If the circle is a helix or spiral or combination, the
  point will include interpolation off the actual circle.
  */
int pmCirclePoint(PmCircle * circle, double angle, PmPose * point)
{
  PmCartesian par, perp;
  double scale;

#ifdef PM_DEBUG
  if (0 == circle ||
      0 == point)
    {
#ifdef PM_PRINT_ERROR
      pmPrintError("error: pmCirclePoint circle or point pointer is null\n");
#endif
      return pmErrno = PM_ERR;
    }
#endif

  /* compute components rel to center */
  pmCartScalMult(circle->rTan, cos(angle), &par);
  pmCartScalMult(circle->rPerp, sin(angle), &perp);

  /* add to get radius vector rel to center */
  pmCartCartAdd(par, perp, &point->tran);

  /* get scale for spiral, helix interpolation */
  if (pm_double_is_zero(circle->angle))
    {
#ifdef PM_PRINT_ERROR
      pmPrintError("error: pmCirclePoint angle is zero\n");
#endif
      return pmErrno = PM_DIV_ERR;
    }
  scale = angle / circle->angle;

  /* add scaled vector in radial dir for spiral */
  pmCartUnit(point->tran, &par);
  pmCartScalMult(par, scale * circle->spiral, &par);
  pmCartCartAdd(point->tran, par, &point->tran);

  /* add scaled vector in helix dir */
  pmCartScalMult(circle->rHelix, scale, &perp);
  pmCartCartAdd(point->tran, perp, &point->tran);

  /* add to center vector for final result */
  pmCartCartAdd(circle->center, point->tran, &point->tran);

  return pmErrno = 0;
}

/*

	DMSDecimalAngleConvert()
	Angle Conversion from Degrees Minutes Seconds to Decimal Degrees
	
	
*/
int pmDMSDecimalAngleConvert( double Degrees, double Minutes, double Seconds, double * Decimal)
{

	// converts a DMS notation to a Decimal notation
	double  temp;
	
	// convert Seconds
	temp = Seconds/60*(1/60); // 1/3600 of a degree
	//convert Minutes
	temp += Minutes/60; // 1/60 of a degree
	// Add Degrees
	temp += Degrees;
	
	
	*Decimal = temp;
	
	
	return 0;
	

}// end DMSDecimalAngleConvert


/*

	3DSpherical2DSphericalConvert()
	
	This fcn converts a 3D spherical position into an "XY" Cartesian coordinate
	on the surface of the Earth.  This is known as a 2D Spherical Coordinate System

*/ 
int pm3DSpherical2DSphericalConvert( double Lon, double Lat, double* X, double * Y)
{

	// Returns a grid coordinate based on True North
	// confies Lon to within +- 180 and 
	// confines Lat to within +- 90 degrees
	// simple conversion 
	const double equator_diameter = 12715.43; // 12715.43 km at the equator

	const double polar_diameter = 12756.274; // 12756.43 along the poles N and S
	//6378137.00 from LLtoUTM

	double circumference_equator;
	double circumference_polar;
	
	// determine circumferences for geoid
	circumference_equator = 2*PM_PI*equator_diameter/2;
	
	circumference_polar = 2*PM_PI*polar_diameter/2;

	// ensure number within 360 degrees
	if (Lon > 360.0 )
	{
		Lon = Lon -360.0;
	}
	
	if ( Lat > 360.0)
	{
		Lat = Lat - 360.0;
	}

	// change Lon to between +- 180 degrees
	if (Lon > 180.00 && Lon <= 360.01)
	{
		Lon = Lon - 360.0;
		
	}	
			
	// change Lat to between +- 90 degrees
	if (Lat > 90.00 && Lat <= 180.01)
	{
		Lat = Lat - 180.0;
		
	}

	// convert Lon to X coord in metres
	*X = Lon/180.0*circumference_equator/2*(1000.0);
	
	// convert Lat to Y coord in metres
	*Y = Lat/90.0*circumference_polar/4*(1000.0);
	
	
	return 0;
}
