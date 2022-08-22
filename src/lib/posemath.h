#ifndef POSEMATH_H
#define POSEMATH_H

/*
  posemath.h

  Declarations for pose math library data types and manipulation functions.

  Data types comprise various representations of translation and rotation
  quantities, and a 'pose' for representing the location and orientation
  of a frame in space relative to a base frame. Translation representations
  include cartesian, spherical, and cylindrical coordinates. All of these
  contain 3 elements. Rotation representations include rotation vectors,
  quaternions, rotation matrices, Euler angles, and roll-pitch-yaw. These
  contain at least 3 elements, and may contain more. Only 3 are necessary
  for the 3 degrees of freedom for either translation or rotation, but some
  data representations use more for computational efficiency or intuition
  at the expense of storage space.

  Types are abbreviated in function naming with a few letters. Functions
  exist for conversion between data types, checking for consistency,
  normalization into consistency, extracting features such as size, and
  arithmetic operations.

  Names of data representations are in all capitals, prefixed with 'PM_'.
  Names of functions are in mixed case, prefixed with 'pm', with case changes
  used to indicate new quantities instead of underscores. Function syntax
  looks like

    int pmQuatRotConvert(PM_QUATERNION, PM_ROTATION_VECTOR *);

  The return value is an error code, 0 for success, or a non-zero error
  code for failure, for example:

    #define PM_ERR -1
    #define PM_IMPL_ERR -2

  The global variable 'pmErrno' is set to this return value.

  C++ classes are used for data types so that operator overloading can
  be used to reduce the programming labor. Using the overloaded operator
  version of functions loses the integer error code. The global
  variable 'pmErrno' can be queried after these operations. This is not
  thread-safe or reentrant.

  C++ names corresponding to the C structures use case mixing instead
  of all caps. Thus, a quaternion in C++ is a PmQuaternion.

  The MATH_DEBUG symbol can be defined to include error reporting via
  printed errors.

  Native efficient C functions exist for the PM_CARTESIAN, PM_QUATERNION,
  and PM_POSE types. Constructors in all the classes have been defined
  to convert to/from PM_CARTESIAN and any other translation type, and
  to convert to/from PM_QUATERNION and any other rotation type. This means
  that if no explicit C functions exist for another type, conversions
  to the corresponding native type will occur automatically. If more
  efficiency is desired for a particular type, C functions to handle the
  operations should be coded and the overloaded C++ functions or operators
  should be added.

  Modification History:
  6-Apr-2005  DRE (from DRDC) Added quaternion conjugate, quaternion addition, and quaternion interpolation including:
    quaternionlerp (linear interpolation) quaternionslerp (linear interpolation) quaternionsquad (spherical cubic interpolation)
  15-Nov-1999 WPS added PM_QUATERNION::PM_QUATERNION(PM_AXIS,angle) and
  PM_QUATERNION::axisAngleMult(PM_AXIS,angle)
  28-Jan-1999  FMP changed FUZZ values to 0.000001
  1-Sep-1998  FMP added PM_CARTESIAN operator * (PM_POSE, PM_CARTESIAN)
  22-Jun-1998 WPS provide definition for pmPrintError even if PM_DEBUG is not
  defined.
  15-Jan-1998  FMP added inverse functions for homogeneous xforms
  5-Jan-1998  FMP added PmAxis enum, pmAxisAngleQuatConvert(),
  pmQuatAngleAxisMult(); removed pmPoseNorm,IsNorm() decls
  18-Dec-1997  FMP changed line, circle to use poses
  9-Oct-1997  FMP added CART_FUZZ
  17-Jul-1997 WPS remove C++ comments for vxworks5.3
  14-Jul-1997  FMP removed #ifdef VXWORKS around copy ctors. Now all
  plats get them.
  10-Jul-1997  FMP switched all C structs from, for example,
  PM_CARTESIAN to PmCartesian, so that C and C++ can coexist without
  weird problems. Now it's explicit.
  19-Jun-1997  FMP added PM_LINE
  18-Jun-1997  FMP added circle utility decls; pmCartPlaneProj()
  16-May-1997 WPS added #ifdef __cplusplus #define __CPLUSPLUS__
  all ANSI C++ compilers define __cplusplus automatically so it
  makes more sense to use than __CPLUSPLUS__ which needs to be
  defined separately.
  7-May-1997 WPS added copy constructors to avoid
  ambigous type error from VxWorks compiler.
  14-Apr-1997  FMP changed to split C/C++
  13-Mar-1997  FMP changed PM_STRICT_NAMESPACE to PM_LOOSE_NAMESPACE since
  leaving it out should result in a stricter namespace
  13-Mar-1997  FMP moved PI, etc. to PM_PI, etc., and moved PI into
  (not) PM_STRICT_NAMESPACE
  7-Mar-1997  FMP inlined default ctor/dtor
   5-Mar-1997 WPS converted all C++ comments to C comments
   for compatibility with sparcworks C  compiler
  12-Feb-1997  FMP removed sincos decl-- it's in sincos.h
  11-Feb-1997  FMP added PM_HOMOGENEOUS
  10-Feb-1997  FMP removed print functions, put into mathprnt.h
  3-Feb-1997  FMP began conversion to PM_ prefix version,
  separating translation vectors from rotation vectors
  3-Feb-1997  FMP added mathMVA_convert
  31-Jan-1997  FMP added more and more stuff
  29-Jan-1997  FMP made it C++, starting with VECTOR, backward-compatible
  via __CPLUSPLUS__ flag with C.
*/
/* $Id: posemath.h,v 1.2 2005/06/06 18:53:07 dave Exp $ */
#include <stdio.h>

#ifdef __cplusplus

#define USE_CONST
#define USE_CCONST
#define USE_REF

#ifdef USE_CCONST
#define PM_CCONST const
#else
#define PM_CCONST
#endif

#ifdef USE_CONST
#define PM_CONST const
#else
#define PM_CONST
#endif

#ifdef USE_REF
#define PM_REF  &
#else
#define PM_REF
#endif

#define INCLUDE_POSEMATH_COPY_CONSTRUCTORS

/* forward declarations-- conversion ctors will need these */

/* translation types */
struct PM_CARTESIAN;       /* Cart */
struct PM_SPHERICAL;       /* Sph */
struct PM_CYLINDRICAL;     /* Cyl */

/* rotation types */
struct PM_ROTATION_VECTOR; /* Rot */
struct PM_ROTATION_MATRIX; /* Mat */
struct PM_QUATERNION;     /* Quat  */
struct PM_EULER_ZYZ;      /* Zyz */
struct PM_EULER_ZYX;      /* Zyx */
struct PM_RPY;            /* Rpy */

/* pose types */
struct PM_POSE;            /* Pose */
struct PM_HOMOGENEOUS;    /* Hom */

/* PM_CARTESIAN */

struct PM_CARTESIAN
{
  /* ctors/dtors */
  PM_CARTESIAN():x(0),y(0),z(0) {};
  PM_CARTESIAN(double _x, double _y, double _z);
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_CARTESIAN(PM_CCONST PM_CARTESIAN &cart); /* added  7-May-1997 by WPS */
#endif

  PM_CARTESIAN(PM_CONST PM_CYLINDRICAL PM_REF c); /* conversion */
  PM_CARTESIAN(PM_CONST PM_SPHERICAL PM_REF s); /* conversion */

  /* operators */
  double & operator [] (int n);             /* this[n] */
  PM_CARTESIAN operator = (PM_CARTESIAN v); /* this = v */

  /* data */
  double x, y, z;                     /* this.x, etc. */
};

/* PM_SPHERICAL */

struct PM_SPHERICAL
{
  /* ctors/dtors */
  PM_SPHERICAL():theta(0),phi(0),r(0){};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_SPHERICAL(PM_CCONST PM_SPHERICAL &s);
#endif
  PM_SPHERICAL(double _theta, double _phi, double _r);
  PM_SPHERICAL(PM_CONST PM_CYLINDRICAL PM_REF v); /* conversion */
  PM_SPHERICAL(PM_CONST PM_CARTESIAN PM_REF v); /* conversion */

  /* operators */
  double & operator [] (int n);             /* this[n] */
  PM_SPHERICAL operator = (PM_SPHERICAL s); /* this = s */

  /* data */
  double theta, phi, r;
};

/* PM_CYLINDRICAL */

struct PM_CYLINDRICAL
{
  /* ctors/dtors */
  PM_CYLINDRICAL():theta(0),r(0),z(0) {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_CYLINDRICAL(PM_CCONST PM_CYLINDRICAL &c);
#endif
  PM_CYLINDRICAL(double _theta, double _r, double _z);
  PM_CYLINDRICAL(PM_CONST PM_CARTESIAN  PM_REF v); /* conversion */
  PM_CYLINDRICAL(PM_CONST PM_SPHERICAL  PM_REF v); /* conversion */

  /* operators */
  double & operator [] (int n);             /* this[n] */
  PM_CYLINDRICAL operator = (PM_CYLINDRICAL c); /* this = c */

  /* data */
  double theta, r, z;
};

/* PM_ROTATION_VECTOR */

struct PM_ROTATION_VECTOR
{
  /* ctors/dtors */
  PM_ROTATION_VECTOR() :s(0),x(0),y(0),z(0){};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_ROTATION_VECTOR(PM_CCONST PM_ROTATION_VECTOR &r);
#endif
  PM_ROTATION_VECTOR(double _r, double _x, double _y, double _z);
  PM_ROTATION_VECTOR(PM_CONST PM_QUATERNION PM_REF q); /* conversion */

  /* operators */
  double &  operator [] (int n);        /* this[n] */
  PM_ROTATION_VECTOR operator = (PM_ROTATION_VECTOR r); /* this = r */

  /* data */
  double s, x, y, z;
};

/* PM_ROTATION_MATRIX */

struct PM_ROTATION_MATRIX
{
  /* ctors/dtors */
  PM_ROTATION_MATRIX():x(),y(),z() {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_ROTATION_MATRIX(PM_CCONST PM_ROTATION_MATRIX &mat); /* added  7-May-1997 by WPS */
#endif
  PM_ROTATION_MATRIX(double xx, double xy, double xz,
                     double yx, double yy, double yz,
                     double zx, double zy, double zz);
  PM_ROTATION_MATRIX(PM_CARTESIAN _x, PM_CARTESIAN _y, PM_CARTESIAN _z);
  PM_ROTATION_MATRIX(PM_CONST PM_ROTATION_VECTOR PM_REF v); /* conversion */
  PM_ROTATION_MATRIX(PM_CONST PM_QUATERNION PM_REF q);      /* conversion */
  PM_ROTATION_MATRIX(PM_CONST PM_EULER_ZYZ PM_REF zyz);     /* conversion */
  PM_ROTATION_MATRIX(PM_CONST PM_EULER_ZYX PM_REF zyx);     /* conversion */
  PM_ROTATION_MATRIX(PM_CONST PM_RPY PM_REF rpy);           /* conversion */

  /* operators */
  PM_CARTESIAN & operator [] (int n);   /* this[n] */
  PM_ROTATION_MATRIX operator = (PM_ROTATION_MATRIX m); /* this = m */

  /* data */
  PM_CARTESIAN x, y, z;
};

/* PM_QUATERNION */

enum PM_AXIS { PM_X, PM_Y, PM_Z};

struct PM_QUATERNION
{
  /* ctors/dtors */
  PM_QUATERNION():s(0),x(0),y(0),z(0) {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_QUATERNION(PM_CCONST PM_QUATERNION &quat); /* added  7-May-1997 by WPS */
#endif
  PM_QUATERNION(double _s, double _x, double _y, double _z);
  PM_QUATERNION(PM_CONST PM_ROTATION_VECTOR PM_REF v); /* conversion */
  PM_QUATERNION(PM_CONST PM_ROTATION_MATRIX PM_REF m); /* conversion */
  PM_QUATERNION(PM_CONST PM_EULER_ZYZ PM_REF zyz);     /* conversion */
  PM_QUATERNION(PM_CONST PM_EULER_ZYX PM_REF zyx);     /* conversion */
  PM_QUATERNION(PM_CONST PM_RPY PM_REF rpy);           /* conversion */
  PM_QUATERNION(PM_AXIS  axis, double angle);  /* conversion */

  /* operators */
  double & operator [] (int n); /* this[n] */
  PM_QUATERNION operator = (PM_QUATERNION q);   /* this = q */

  /* functions */
  void axisAngleMult(PM_AXIS axis, double angle);

  /* data */
  double s, x, y, z;            /* this.s, etc. */
};

/* PM_EULER_ZYZ */

struct PM_EULER_ZYZ
{
  /* ctors/dtors */
  PM_EULER_ZYZ() :z(0),y(0),zp(0) {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_EULER_ZYZ(PM_CCONST PM_EULER_ZYZ &zyz);
#endif
  PM_EULER_ZYZ(double _z, double _y, double _zp);
  PM_EULER_ZYZ(PM_CONST PM_QUATERNION PM_REF q); /* conversion */
  PM_EULER_ZYZ(PM_CONST PM_ROTATION_MATRIX PM_REF m); /* conversion */

  /* operators */
  double & operator [] (int n);
  PM_EULER_ZYZ operator = (PM_EULER_ZYZ zyz);

  /* data */
  double z, y, zp;
};

/* PM_EULER_ZYX */

struct PM_EULER_ZYX
{
  /* ctors/dtors */
  PM_EULER_ZYX():z(0),y(0),x(0) {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_EULER_ZYX(PM_CCONST PM_EULER_ZYX &zyx);
#endif
  PM_EULER_ZYX(double _z, double _y, double _x);
  PM_EULER_ZYX(PM_CONST PM_QUATERNION PM_REF q); /* conversion */
  PM_EULER_ZYX(PM_CONST PM_ROTATION_MATRIX PM_REF m); /* conversion */

  /* operators */
  double & operator [] (int n);
  PM_EULER_ZYX operator = (PM_EULER_ZYX zyx);

  /* data */
  double z, y, x;
};

/* PM_RPY */

struct PM_RPY
{
  /* ctors/dtors */
  PM_RPY() :r(0),p(0),y(0){};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_RPY(PM_CCONST PM_RPY &rpy);                /* added  7-May-1997 by WPS */
#endif
  PM_RPY(double _r, double _p, double _y);
  PM_RPY(PM_CONST PM_QUATERNION  PM_REF q);     /* conversion */
  PM_RPY(PM_CONST PM_ROTATION_MATRIX PM_REF m); /* conversion */

  /* operators */
  double & operator [] (int n);
  PM_RPY operator = (PM_RPY rpy);

  /* data */
  double r, p, y;
};

/* PM_POSE */

struct PM_POSE
{
  /* ctors/dtors */
  PM_POSE() :tran(),rot() {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_POSE(PM_CCONST PM_POSE &p);
#endif
  PM_POSE(PM_CARTESIAN v, PM_QUATERNION q);
  PM_POSE(double x, double y, double z,
          double s, double sx, double sy, double sz);
  PM_POSE(PM_CONST PM_HOMOGENEOUS PM_REF h);    /* conversion */

  /* operators */
  double & operator [] (int n); /* this[n] */
  PM_POSE operator = (PM_POSE p);       /* this = p */

  /* data */
  PM_CARTESIAN tran;
  PM_QUATERNION rot;
};

/* PM_HOMOGENEOUS */

struct PM_HOMOGENEOUS
{
  /* ctors/dtors */
  PM_HOMOGENEOUS():tran(),rot() {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_HOMOGENEOUS(PM_CCONST PM_HOMOGENEOUS &h);
#endif
  PM_HOMOGENEOUS(PM_CARTESIAN v, PM_ROTATION_MATRIX m);
  PM_HOMOGENEOUS(PM_CONST PM_POSE PM_REF  p);   /* conversion */

  /* operators */
  PM_CARTESIAN & operator [] (int n); /* column vector */
  PM_HOMOGENEOUS operator = (PM_HOMOGENEOUS h);

  /* data ( [ 0 0 0 1 ] element is manually returned by [] if needed ) */
  PM_CARTESIAN tran;
  PM_ROTATION_MATRIX rot;
};

/* PM_LINE */

struct PM_LINE
{
  /* ctors/dtors */
  PM_LINE():start(),end(),uVec() {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_LINE(PM_CCONST PM_LINE &);
#endif

  /* functions */
  int init(PM_POSE start, PM_POSE end);
  int point(double len, PM_POSE * point);

  /* data */
  PM_POSE start;                /* where motion was started */
  PM_POSE end;          /* where motion is going */
  PM_CARTESIAN uVec;             /* unit vector from start to end */
};

/* PM_CIRCLE */

struct PM_CIRCLE
{
  /* ctors/dtors */
  PM_CIRCLE():center(),normal(),rTan(),rPerp(),rHelix(),radius(0),angle(0),spiral(0) {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
  PM_CIRCLE(PM_CCONST PM_CIRCLE &);
#endif

  /* functions */
  int init(PM_POSE start, PM_POSE end,
           PM_CARTESIAN center, PM_CARTESIAN normal,
           int turn);
  int point(double angle, PM_POSE * point);

  /* data */
  PM_CARTESIAN center;
  PM_CARTESIAN normal;
  PM_CARTESIAN rTan;
  PM_CARTESIAN rPerp;
  PM_CARTESIAN rHelix;
  double radius;
  double angle;
  double spiral;
};

/* overloaded external functions */

/* dot */
extern double dot(PM_CARTESIAN v1, PM_CARTESIAN v2);

/* cross */
extern PM_CARTESIAN cross(PM_CARTESIAN v1, PM_CARTESIAN v2);


/* norm */
extern PM_CARTESIAN norm(PM_CARTESIAN v);
extern PM_QUATERNION norm(PM_QUATERNION q);
extern PM_ROTATION_VECTOR norm(PM_ROTATION_VECTOR r);
extern PM_ROTATION_MATRIX norm(PM_ROTATION_MATRIX m);


/* unit */
extern PM_CARTESIAN unit(PM_CARTESIAN v);
extern PM_QUATERNION unit(PM_QUATERNION q);
extern PM_ROTATION_VECTOR unit(PM_ROTATION_VECTOR r);
extern PM_ROTATION_MATRIX unit(PM_ROTATION_MATRIX m);


/* isNorm */
extern int isNorm(PM_CARTESIAN v);
extern int isNorm(PM_QUATERNION q);
extern int isNorm(PM_ROTATION_VECTOR r);
extern int isNorm(PM_ROTATION_MATRIX m);

/* mag */
extern double mag(PM_CARTESIAN v);

/* disp */
extern double disp(PM_CARTESIAN v1, PM_CARTESIAN v2);

/* inv */
extern PM_CARTESIAN inv(PM_CARTESIAN v);
extern PM_ROTATION_MATRIX inv(PM_ROTATION_MATRIX m);
extern PM_QUATERNION inv(PM_QUATERNION q);
extern PM_POSE inv(PM_POSE p);
extern PM_HOMOGENEOUS inv(PM_HOMOGENEOUS h);

/* project */
extern PM_CARTESIAN proj(PM_CARTESIAN v1, PM_CARTESIAN v2);

/* overloaded arithmetic functions */

/* unary +, - for translation, rotation, pose */
extern PM_CARTESIAN operator + (PM_CARTESIAN v);
extern PM_CARTESIAN operator - (PM_CARTESIAN v);
extern PM_QUATERNION operator + (PM_QUATERNION q);
extern PM_QUATERNION operator - (PM_QUATERNION q);
extern PM_POSE operator + (PM_POSE p);
extern PM_POSE operator - (PM_POSE p);

/* compare operators */
extern int operator == (PM_CARTESIAN v1, PM_CARTESIAN v2);
extern int operator == (PM_QUATERNION q1, PM_QUATERNION q2);
extern int operator == (PM_POSE p1, PM_POSE p2);
extern int operator != (PM_CARTESIAN v1, PM_CARTESIAN v2);
extern int operator != (PM_QUATERNION q1, PM_QUATERNION q2);
extern int operator != (PM_POSE p1, PM_POSE p2);

/* translation +, -, scalar *, - */

/* v + v */
extern PM_CARTESIAN operator + (PM_CARTESIAN v1, PM_CARTESIAN v2);
/* v - v */
extern PM_CARTESIAN operator - (PM_CARTESIAN v1, PM_CARTESIAN v2);
/* v * s */
extern PM_CARTESIAN operator * (PM_CARTESIAN v, double s);
/* s * v */
extern PM_CARTESIAN operator * (double s, PM_CARTESIAN v);
/* v / s */
extern PM_CARTESIAN operator / (PM_CARTESIAN v, double s);

/* rotation * by scalar, translation, and rotation */

/* s * q */
extern PM_QUATERNION operator * (double s, PM_QUATERNION q);
/* q * s */
extern PM_QUATERNION operator * ( PM_QUATERNION q, double s);
/* q / s */
extern PM_QUATERNION operator / ( PM_QUATERNION q, double s);
/* q * v */
extern PM_CARTESIAN operator * (PM_QUATERNION q, PM_CARTESIAN v);
/* q * q */
extern PM_QUATERNION operator * (PM_QUATERNION q1, PM_QUATERNION q2);
/* m * m */
extern PM_ROTATION_MATRIX operator * (PM_ROTATION_MATRIX m1, PM_ROTATION_MATRIX m2);

/* pose operators */

/* q * p */
extern PM_POSE operator * (PM_QUATERNION q, PM_POSE p);
/* p * p */
extern PM_POSE operator * (PM_POSE p1, PM_POSE p2);
/* p * v */
extern PM_CARTESIAN operator * (PM_POSE p, PM_CARTESIAN v);

#endif /* __cplusplus */

/* now comes the C stuff */

#ifdef __cplusplus
extern "C" {
#endif

/* PmCartesian */

typedef struct
{
  double x, y, z;                     /* this.x, etc. */

} PmCartesian;

/* PmSpherical */

typedef struct
{
  double theta, phi, r;

} PmSpherical;

/* PmCylindrical */

typedef struct
{
  double theta, r, z;

} PmCylindrical;

/* PmAxis */
#ifdef __cplusplus
typedef PM_AXIS PmAxis;
#else
typedef enum {PM_X, PM_Y, PM_Z} PmAxis;
#endif

/* PmRotationVector */

typedef struct
{
  double s, x, y, z;

} PmRotationVector;

/* PmRotationMatrix */

typedef struct
{
  PmCartesian x, y, z;

} PmRotationMatrix;

/* PmQuaternion */

typedef struct
{
  double s, x, y, z;            /* this.s, etc. */

} PmQuaternion;

/* PmEulerZyz */

typedef struct
{
  double z, y, zp;

} PmEulerZyz;

/* PmEulerZyx */

typedef struct
{
  double z, y, x;

} PmEulerZyx;

/* PmRpy */

typedef struct
{
  double r, p, y;

} PmRpy;

/* PmPose */

typedef struct
{
  PmCartesian tran;
  PmQuaternion rot;

} PmPose;

/* Homogeneous transform PmHomogeneous */

typedef struct
{
  PmCartesian tran;
  PmRotationMatrix rot;

} PmHomogeneous;

/* line structure */

typedef struct
{
  PmPose start;         	/* where motion was started */
  PmPose end;           	/* where motion is going */
  PmCartesian uVec;            	/* unit vector from start to end */
  PmQuaternion qVec;		/* unit of rotation */
  double tmag;
  double rmag;
  int tmag_is_greater_than_rmag;
  int tmag_zero;
  int rmag_zero;

} PmLine;

/* Generalized circle structure */

typedef struct
{
  PmCartesian center;
  PmCartesian normal;
  PmCartesian rTan;
  PmCartesian rPerp;
  PmCartesian rHelix;
  double radius;
  double angle;
  double spiral;

} PmCircle;

/*
   shorthand types for normal use-- don't define PM_LOOSE_NAMESPACE if these
   names are used by other headers you need to include and you don't want
   these shorthand versions
*/

/* some nice constants */

#define PM_PI      3.14159265358979323846
#define PM_PI_2    1.57079632679489661923
#define PM_PI_4    0.78539816339744830962
#define PM_2_PI    6.28318530717958647692

#ifdef PM_LOOSE_NAMESPACE

typedef PmCartesian       VECTOR;
typedef PmSpherical       SPHERICAL;
typedef PmCylindrical     CYLINDRICAL;
typedef PmQuaternion      QUATERNION;
typedef PmRotationMatrix  MATRIX;
typedef PmEulerZyz        ZYZ;
typedef PmEulerZyx        ZYX;
typedef PmRpy             RPY;
typedef PmPose            POSE;
typedef PmHomogeneous     HX;
typedef PmCircle          CIRCLE;
typedef PmLine            LINE;

#define PI                PM_PI
#define PI_2              PM_PI_2
#define PI_4              PM_PI_4
#define TWO_PI            PM_2_PI /* 2_PI invalid macro name */
#endif

/* quicky macros */

#define pmClose(a, b, eps) ((fabs((a) - (b)) < (eps)) ? 1 : 0)
#define pmSq(x) ((x)*(x))

#ifdef TO_DEG
#undef TO_DEG
#endif
#define TO_DEG (180./PM_PI)

#ifdef TO_RAD
#undef TO_RAD
#endif
#define TO_RAD (PM_PI/180.)

/* FIXME-- fix these */

/* DOUBLE_FUZZ is the smallest double, d, such that (1+d != 1) w/o FPC.
   DOUBLECP_FUZZ is the same only with the Floating Point CoProcessor */

#define DOUBLE_FUZZ 2.2204460492503131e-16
#define DOUBLECP_FUZZ 1.0842021724855044e-19

#define CART_FUZZ (0.000001)
/* how close a cartesian vector's magnitude must be for it to be considered
   a zero vector */

#define Q_FUZZ (.000001)
/* how close elements of a Q must be to be equal */

#define QS_FUZZ (.000001)
/* how close q.s is to 0 to be 180 deg rotation */

#define RS_FUZZ (.000001)
/* how close r.s is for a rotation vector to be considered 0 */

#define QSIN_FUZZ (.000001)
/* how close sin(a/2) is to 0 to be zero rotat */

#define V_FUZZ (.000001)
/* how close elements of a V must be to be equal */

#define SQRT_FUZZ (-.000001)
/* how close to 0 before math_sqrt() is error */

#define UNIT_VEC_FUZZ (.000001)
/* how close mag of vec must be to 1.00 */

#define UNIT_QUAT_FUZZ (.000001)
/* how close mag of quat must be to 1.00 */

#define UNIT_SC_FUZZ (.000001)
/* how close mag of sin, cos must be to 1.00 */

#define E_EPSILON (.000001)
/* how close second ZYZ euler angle must be to 0/PI for degeneration */

#define SINGULAR_EPSILON (.000001)
/* how close to zero the determinate of a matrix must be for singularity */

#define RPY_P_FUZZ (0.000001)
/* how close pitch is to zero for RPY to degenerate */

#define ZYZ_Y_FUZZ (0.000001)
/* how close Y is to zero for ZYZ Euler to degenerate */

#define ZYX_Y_FUZZ (0.000001)
/* how close Y is to zero for ZYX Euler to degenerate */

#define CIRCLE_FUZZ (0.000001)
/* Bug fix for the missing circles problem */

/* debug output printing */
extern void pmPrintError(const char *fmt, ...);

/* global error number and errors */
extern int pmErrno;
extern void pmPerror(const char *fmt);
#define PM_ERR             -1   /* unspecified error */
#define PM_IMPL_ERR        -2   /* not implemented */
#define PM_NORM_ERR        -3   /* arg should have been norm */
#define PM_DIV_ERR         -4   /* divide by zero error */

/* Scalar functions */

extern double pmSqrt(double x);

/* Translation rep conversion functions */

extern int pmCartSphConvert(PmCartesian, PmSpherical *);
extern int pmCartCylConvert(PmCartesian, PmCylindrical *);
extern int pmSphCartConvert(PmSpherical, PmCartesian *);
extern int pmSphCylConvert(PmSpherical, PmCylindrical *);
extern int pmCylCartConvert(PmCylindrical, PmCartesian *);
extern int pmCylSphConvert(PmCylindrical, PmSpherical *);

/* Rotation rep conversion functions */

extern int pmAxisAngleQuatConvert(PmAxis, double, PmQuaternion *);

extern int pmRotQuatConvert(PmRotationVector, PmQuaternion *);
extern int pmRotMatConvert(PmRotationVector, PmRotationMatrix *);
extern int pmRotZyzConvert(PmRotationVector, PmEulerZyz *);
extern int pmRotZyxConvert(PmRotationVector, PmEulerZyx *);
extern int pmRotRpyConvert(PmRotationVector, PmRpy *);

extern int pmQuatRotConvert(PmQuaternion, PmRotationVector *);
extern int pmQuatMatConvert(PmQuaternion, PmRotationMatrix *);
extern int pmQuatZyzConvert(PmQuaternion, PmEulerZyz *);
extern int pmQuatZyxConvert(PmQuaternion, PmEulerZyx *);
extern int pmQuatRpyConvert(PmQuaternion, PmRpy *);

extern int pmMatRotConvert(PmRotationMatrix, PmRotationVector *);
extern int pmMatQuatConvert(PmRotationMatrix, PmQuaternion *);
extern int pmMatZyzConvert(PmRotationMatrix, PmEulerZyz *);
extern int pmMatZyxConvert(PmRotationMatrix, PmEulerZyx *);
extern int pmMatRpyConvert(PmRotationMatrix, PmRpy *);

extern int pmZyzRotConvert(PmEulerZyz, PmRotationVector *);
extern int pmZyzQuatConvert(PmEulerZyz, PmQuaternion *);
extern int pmZyzMatConvert(PmEulerZyz, PmRotationMatrix *);
extern int pmZyzZyxConvert(PmEulerZyz, PmEulerZyx *);
extern int pmZyzRpyConvert(PmEulerZyz, PmRpy *);

extern int pmZyxRotConvert(PmEulerZyx, PmRotationVector *);
extern int pmZyxQuatConvert(PmEulerZyx, PmQuaternion *);
extern int pmZyxMatConvert(PmEulerZyx, PmRotationMatrix *);
extern int pmZyxZyzConvert(PmEulerZyx, PmEulerZyz *);
extern int pmZyxRpyConvert(PmEulerZyx, PmRpy *);

extern int pmRpyRotConvert(PmRpy, PmRotationVector *);
extern int pmRpyQuatConvert(PmRpy, PmQuaternion *);
extern int pmRpyMatConvert(PmRpy, PmRotationMatrix *);
extern int pmRpyZyzConvert(PmRpy, PmEulerZyz *);
extern int pmRpyZyxConvert(PmRpy, PmEulerZyx *);

/* Combined rep conversion functions */

extern int pmPoseHomConvert(PmPose p, PmHomogeneous * h);

extern int pmHomPoseConvert(PmHomogeneous h, PmPose * p);

/* Arithmetic functions

   Note: currently, only functions for PmCartesian, PmQuaternion, and
   PmPose are supported directly. The type conversion functions
   will be used implicitly when applying arithmetic function
   to other types. This will be slower and less accurate. Explicit
   functions can be added incrementally.
*/

/* translation functions */

/* NOTE:  only Cartesian type supported in C now */

extern int pmCartCartCompare(PmCartesian, PmCartesian);
extern int pmCartCartDot(PmCartesian, PmCartesian, double *);
extern int pmCartCartCross(PmCartesian, PmCartesian, PmCartesian *);
extern int pmCartMag(PmCartesian, double *);
extern int pmCartCartDisp(PmCartesian v1, PmCartesian v2, double *d);
extern int pmCartCartAdd(PmCartesian, PmCartesian, PmCartesian *);
extern int pmCartCartSub(PmCartesian, PmCartesian, PmCartesian *);
extern int pmCartScalMult(PmCartesian, double, PmCartesian *);
extern int pmCartScalDiv(PmCartesian, double, PmCartesian *);
extern int pmCartNeg(PmCartesian, PmCartesian *);
extern int pmCartUnit(PmCartesian v, PmCartesian * vout);
#if 0
extern int pmCartNorm(PmCartesian v, PmCartesian * vout);
#else
/* Hopefully guaranteed to cause a compile error when used. */
#define pmCartNorm(a,b,c,d,e)  bad{a.b.c.d.e}
#endif

extern int pmCartIsNorm(PmCartesian v);
extern int pmCartInv(PmCartesian, PmCartesian *);
extern int pmCartCartProj(PmCartesian, PmCartesian, PmCartesian *);
extern int pmCartPlaneProj(PmCartesian v, PmCartesian normal, PmCartesian * vout);

/* rotation functions */

/* quaternion functions */
extern int pmQuatQuatAdd(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout);
extern int pmQuatQuatCompare(PmQuaternion, PmQuaternion);
extern int pmQuatQuatDotProduct(PmQuaternion q1, PmQuaternion q2, double * dot);
extern int pmQuatQuatLerp(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout, double h);
extern int pmQuatQuatSlerp(PmQuaternion q1, PmQuaternion q2, PmQuaternion * qout, double h);
extern int pmQuatQuatSquad(PmQuaternion q1, PmQuaternion q2, PmQuaternion q3, PmQuaternion q4, PmQuaternion * qout, double h);
extern int pmQuatConjugate(PmQuaternion q1, PmQuaternion * qout);
extern int pmQuatMag(PmQuaternion q, double * d);
extern int pmQuatNorm(PmQuaternion, PmQuaternion *);
extern int pmQuatInv(PmQuaternion, PmQuaternion *);
extern int pmQuatIsNorm(PmQuaternion);
extern int pmQuatScalMult(PmQuaternion q, double s, PmQuaternion * qout);
extern int pmQuatScalDiv(PmQuaternion q, double s, PmQuaternion * qout);
extern int pmQuatQuatMult(PmQuaternion, PmQuaternion, PmQuaternion *);
extern int pmQuatQuatMult(PmQuaternion, PmQuaternion, PmQuaternion *);
extern int pmQuatCartMult(PmQuaternion, PmCartesian, PmCartesian *);
extern int pmQuatAxisAngleMult(PmQuaternion, PmAxis, double, PmQuaternion *);

/* rotation vector functions */

extern int pmRotScalMult(PmRotationVector, double, PmRotationVector *);
extern int pmRotScalDiv(PmRotationVector, double, PmRotationVector *);
extern int pmQuatQuatDistance(PmQuaternion q1, PmQuaternion q2, double * result);
extern int pmRotIsNorm(PmRotationVector);
extern int pmRotNorm(PmRotationVector, PmRotationVector *);

/* rotation matrix functions */

/*        |  m.x.x   m.y.x   m.z.x  |   */
/*   M =  |  m.x.y   m.y.y   m.z.y  |   */
/*        |  m.x.z   m.y.z   m.z.z  |   */

extern int pmMatNorm(PmRotationMatrix m, PmRotationMatrix * mout);
extern int pmMatIsNorm(PmRotationMatrix m);
extern int pmMatInv(PmRotationMatrix m, PmRotationMatrix *mout);
extern int pmMatCartMult(PmRotationMatrix m, PmCartesian v, PmCartesian * vout);
extern int pmMatMatMult(PmRotationMatrix m1, PmRotationMatrix m2, PmRotationMatrix * mout);

/* pose functions*/

extern int pmPosePoseCompare(PmPose, PmPose);
extern int pmPoseInv(PmPose p, PmPose *);
extern int pmPoseCartMult(PmPose, PmCartesian, PmCartesian *);
extern int pmPosePoseMult(PmPose, PmPose, PmPose *);

/* homogeneous functions */
extern int pmHomInv(PmHomogeneous, PmHomogeneous *);

/* line functions */

extern int pmLineInit(PmLine * line,
                      PmPose start, PmPose end);
extern int pmLinePoint(PmLine * line,
                       double len, PmPose * point);

/* circle functions */

extern int pmCircleInit(PmCircle * circle,
                        PmPose start, PmPose end,
                        PmCartesian center, PmCartesian normal,
                        int turn);
extern int pmCirclePoint(PmCircle * circle,
                         double angle, PmPose * point);

/* slicky macros for item-by-item copying between C and C++ structs */

#define toCart(src,dst) {(dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z;}

#define toCyl(src,dst) {(dst)->theta = (src).theta; (dst)->r = (src).r; (dst)->z = (src).z;}

#define toSph(src,dst) {(dst)->theta = (src).theta; (dst)->phi = (src).phi; (dst)->r = (src).r;}

#define toQuat(src,dst) {(dst)->s = (src).s; (dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z;}

#define toRot(src,dst) {(dst)->s = (src).s; (dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z;}

#define toMat(src,dst) {toCart((src).x, &((dst)->x)); toCart((src).y, &((dst)->y)); toCart((src).z, &((dst)->z));}

#define toEulerZyz(src,dst) {(dst)->z = (src).z; (dst)->y = (src).y; (dst)->zp = (src).zp;}

#define toEulerZyx(src,dst) {(dst)->z = (src).z; (dst)->y = (src).y; (dst)->x = (src).x;}

#define toRpy(src,dst) {(dst)->r = (src).r; (dst)->p = (src).p; (dst)->y = (src).y;}

#define toPose(src,dst) {toCart((src).tran, &((dst)->tran)); toQuat((src).rot, &((dst)->rot));}

#define toHom(src,dst) {toCart((src).tran, &((dst)->tran)); toMat((src).rot, &((dst)->rot));}

#define toLine(src,dst) {toPose((src).start, &((dst)->start)); toPose((src).end, &((dst)->end)); toCart((src).uVec, &((dst)->uVec));}

#define toCircle(src,dst) {toCart((src).center, &((dst)->center)); toCart((src).normal, &((dst)->normal)); toCart((src).rTan, &((dst)->rTan)); toCart((src).rPerp, &((dst)->rPerp)); toCart((src).rHelix, &((dst)->rHelix)); (dst)->radius = (src).radius; (dst)->angle = (src).angle; (dst)->spiral = (src).spiral;}



// coordinate conversion functions
int pmDMSDecimalAngleConvert( double Degrees, double Minutes, double Seconds, double * Decimal);
int pm3DSpherical2DSphericalConvert( double Lon, double Lat, double* X, double * Y);






#ifdef __cplusplus
} /* matches extern "C" for C++ */
#endif

#endif /* #ifndef POSEMATH_H */