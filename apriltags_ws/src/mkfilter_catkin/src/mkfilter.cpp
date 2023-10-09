/* mkfilter -- given n, compute recurrence relation
   to implement Butterworth, Bessel or Chebyshev filter of order n
   A.J. Fisher, University of York   <fisher@minster.york.ac.uk>
   September 1992 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdexcept>
#include <vector>

#include "complex.h"

#define unless(x)   if(!(x))
#define until(x)    while(!(x))

#define VERSION	    "4.6"
#undef	PI
#define PI	    3.14159265358979323846  /* Microsoft C++ does not define M_PI ! */
#define TWOPI	    (2.0 * PI)
#define EPS	    1e-10
#define MAXORDER    10
#define MAXPZ	    512	    /* .ge. 2*MAXORDER, to allow for doubling of poles in BP filter;
			       high values needed for FIR filters */
#define MAXSTRING   256

typedef void (*proc)();
typedef unsigned int uint;

inline double sqr(double x)	    { return x*x;			       }
inline bool seq(const char *s1, const char *s2) { return strcmp(s1,s2) == 0;	       }
inline bool onebit(uint m)	    { return (m != 0) && ((m & (m-1)) == 0);     }

inline double asinh(double x)
  { /* Microsoft C++ does not define */
    return log(x + sqrt(1.0 + sqr(x)));
  }

inline double fix(double x)
  { /* nearest integer */
    return (x >= 0.0) ? floor(0.5+x) : -floor(0.5-x);
  }

#define opt_be 0x00001	/* -Be		Bessel characteristic	       */
#define opt_bu 0x00002	/* -Bu		Butterworth characteristic     */
#define opt_ch 0x00004	/* -Ch		Chebyshev characteristic       */
#define opt_re 0x00008	/* -Re		Resonator		       */
#define opt_pi 0x00010	/* -Pi		proportional-integral	       */

#define opt_lp 0x00020	/* -Lp		lowpass			       */
#define opt_hp 0x00040	/* -Hp		highpass		       */
#define opt_bp 0x00080	/* -Bp		bandpass		       */
#define opt_bs 0x00100	/* -Bs		bandstop		       */
#define opt_ap 0x00200	/* -Ap		allpass			       */

#define opt_a  0x00400	/* -a		alpha value		       */
#define opt_l  0x00800	/* -l		just list filter parameters    */
#define opt_o  0x01000	/* -o		order of filter		       */
#define opt_p  0x02000	/* -p		specified poles only	       */
#define opt_w  0x04000	/* -w		don't pre-warp		       */
#define opt_z  0x08000	/* -z		use matched z-transform	       */
#define opt_Z  0x10000	/* -Z		additional zero		       */

struct pzrep
  { complex poles[MAXPZ], zeros[MAXPZ];
    int numpoles, numzeros;
  };

//Variables for thread safety
struct Data_t {
    pzrep splane, zplane;
    int order;
    double raw_alpha1, raw_alpha2, raw_alphaz;
    complex dc_gain, fc_gain, hf_gain;
    uint options;
    double warped_alpha1, warped_alpha2, chebrip, qfactor;
    bool infq;
    uint polemask;
    double xcoeffs[MAXPZ+1], ycoeffs[MAXPZ+1];
    bool optsok;
};

const static c_complex bessel_poles[] =
  { /* table produced by /usr/fisher/bessel --	N.B. only one member of each C.Conj. pair is listed */
    { -1.00000000000e+00, 0.00000000000e+00}, { -1.10160133059e+00, 6.36009824757e-01},
    { -1.32267579991e+00, 0.00000000000e+00}, { -1.04740916101e+00, 9.99264436281e-01},
    { -1.37006783055e+00, 4.10249717494e-01}, { -9.95208764350e-01, 1.25710573945e+00},
    { -1.50231627145e+00, 0.00000000000e+00}, { -1.38087732586e+00, 7.17909587627e-01},
    { -9.57676548563e-01, 1.47112432073e+00}, { -1.57149040362e+00, 3.20896374221e-01},
    { -1.38185809760e+00, 9.71471890712e-01}, { -9.30656522947e-01, 1.66186326894e+00},
    { -1.68436817927e+00, 0.00000000000e+00}, { -1.61203876622e+00, 5.89244506931e-01},
    { -1.37890321680e+00, 1.19156677780e+00}, { -9.09867780623e-01, 1.83645135304e+00},
    { -1.75740840040e+00, 2.72867575103e-01}, { -1.63693941813e+00, 8.22795625139e-01},
    { -1.37384121764e+00, 1.38835657588e+00}, { -8.92869718847e-01, 1.99832584364e+00},
    { -1.85660050123e+00, 0.00000000000e+00}, { -1.80717053496e+00, 5.12383730575e-01},
    { -1.65239648458e+00, 1.03138956698e+00}, { -1.36758830979e+00, 1.56773371224e+00},
    { -8.78399276161e-01, 2.14980052431e+00}, { -1.92761969145e+00, 2.41623471082e-01},
    { -1.84219624443e+00, 7.27257597722e-01}, { -1.66181024140e+00, 1.22110021857e+00},
    { -1.36069227838e+00, 1.73350574267e+00}, { -8.65756901707e-01, 2.29260483098e+00},
  };

static void readcmdline(char*[], struct Data_t& data);
static uint decodeoptions(char*), optbit(char);
static double getfarg(char*);
static int getiarg(char*);
static void usage(), checkoptions(struct Data_t& data), opterror(struct Data_t& data, const char*, int = 0, int = 0), setdefaults(struct Data_t& data);
static void compute_s(struct Data_t& data), choosepole(struct Data_t& data, complex), prewarp(struct Data_t& data), normalize(struct Data_t& data), compute_z_blt(struct Data_t& data);
static complex blt(complex);
static void compute_z_mzt(struct Data_t& data);
static void compute_notch(struct Data_t& data), compute_apres(struct Data_t& data);
static complex reflect(complex);
static void compute_bpres(struct Data_t& data), add_extra_zero(struct Data_t& data);
static void expandpoly(struct Data_t& data), expand(complex[], int, complex[]), multin(complex, int, complex[]);


void mkfilter_process(
    int argc, 
    char** argv, 
    std::vector<double>& argCoefIn, 
    std::vector<double>& argCoefOut, 
    double& argGain)
  { 
    (void)argc;
    struct Data_t data;
    readcmdline(argv, data);
    checkoptions(data);
    setdefaults(data);
    if (data.options & opt_re)
      { if (data.options & opt_bp) compute_bpres(data);	   /* bandpass resonator	 */
	if (data.options & opt_bs) compute_notch(data);	   /* bandstop resonator (notch) */
	if (data.options & opt_ap) compute_apres(data);	   /* allpass resonator		 */
      }
    else
      { if (data.options & opt_pi)
	  { prewarp(data);
	    data.splane.poles[0] = 0.0;
	    data.splane.zeros[0] = -TWOPI * data.warped_alpha1;
	    data.splane.numpoles = data.splane.numzeros = 1;
	  }
	else
	  { compute_s(data);
	    prewarp(data);
	    normalize(data);
	  }
	if (data.options & opt_z) compute_z_mzt(data); else compute_z_blt(data);
      }
    if (data.options & opt_Z) add_extra_zero(data);
    expandpoly(data);

    //Assign results
    complex gain = (data.options & opt_pi) ? data.hf_gain :
        (data.options & opt_lp) ? data.dc_gain :
        (data.options & opt_hp) ? data.hf_gain :
        (data.options & (opt_bp | opt_ap)) ? data.fc_gain :
        (data.options & opt_bs) ? csqrt(data.dc_gain * data.hf_gain) : complex(1.0);
    argGain = hypot(gain);
    argCoefIn.assign(data.zplane.numzeros+1, 0.0);
    argCoefOut.assign(data.zplane.numpoles+1, 0.0);
    for (size_t i=0;i<(size_t)data.zplane.numzeros+1;i++) {
        argCoefIn[data.zplane.numzeros-i] = data.xcoeffs[i];
    }
    for (size_t i=0;i<(size_t)data.zplane.numpoles;i++) {
        argCoefOut[data.zplane.numpoles-i] = data.ycoeffs[i];
    }
  }

static void readcmdline(char *argv[], struct Data_t& data)
  { data.options = 0;
    data.order = 0;
    data.polemask = 0;
    int ap = 0;
    unless (argv[ap] == NULL) ap++; /* skip program name */
    until (argv[ap] == NULL)
      { uint m = decodeoptions(argv[ap++]);
	if (m & opt_ch) data.chebrip = getfarg(argv[ap++]);
	if (m & opt_a)
	  { data.raw_alpha1 = getfarg(argv[ap++]);
	    data.raw_alpha2 = (argv[ap] != NULL && argv[ap][0] != '-') ? getfarg(argv[ap++]) : data.raw_alpha1;
	  }
	if (m & opt_Z) data.raw_alphaz = getfarg(argv[ap++]);
	if (m & opt_o) data.order = getiarg(argv[ap++]);
	if (m & opt_p)
	  { while (argv[ap] != NULL && argv[ap][0] >= '0' && argv[ap][0] <= '9')
	      { int p = atoi(argv[ap++]);
		if (p < 0 || p > 31) p = 31; /* out-of-range value will be picked up later */
		data.polemask |= (1 << p);
	      }
	  }
	if (m & opt_re)
	  { char *s = argv[ap++];
	    if (s != NULL && seq(s,"Inf")) data.infq = true;
	    else { data.qfactor = getfarg(s); data.infq = false; }
	  }
	data.options |= m;
      }
  }

static uint decodeoptions(char *s)
  { unless (*(s++) == '-') usage();
    uint m = 0;
    if (seq(s,"Be")) m |= opt_be;
    else if (seq(s,"Bu")) m |= opt_bu;
    else if (seq(s, "Ch")) m |= opt_ch;
    else if (seq(s, "Re")) m |= opt_re;
    else if (seq(s, "Pi")) m |= opt_pi;
    else if (seq(s, "Lp")) m |= opt_lp;
    else if (seq(s, "Hp")) m |= opt_hp;
    else if (seq(s, "Bp")) m |= opt_bp;
    else if (seq(s, "Bs")) m |= opt_bs;
    else if (seq(s, "Ap")) m |= opt_ap;
    else
      { until (*s == '\0')
	  { uint bit = optbit(*(s++));
	    if (bit == 0) usage();
	    m |= bit;
	  }
      }
    return m;
  }

static uint optbit(char c)
  { switch (c)
      { default:    return 0;
	case 'a':   return opt_a;
	case 'l':   return opt_l;
	case 'o':   return opt_o;
	case 'p':   return opt_p;
	case 'w':   return opt_w;
	case 'z':   return opt_z;
	case 'Z':   return opt_Z;
      }
  }

static double getfarg(char *s)
  { if (s == NULL) usage();
    return atof(s);
  }

static int getiarg(char *s)
  { if (s == NULL) usage();
    return atoi(s);
  }

static void usage()
  { fprintf(stderr, "Mkfilter V.%s from <fisher@minster.york.ac.uk>\n", VERSION);
    fprintf(stderr, "Usage: mkfilter [-Be | -Bu | -Ch <r> | -Pi] [-Lp | -Hp | -Bp | -Bs] [-p <n1> <n2> ...] [-{lwz}] "
				     "[-Z <alphaz>] "
				     "-o <order> -a <alpha1> [ <alpha2> ]\n");
    fprintf(stderr, "       mkfilter -Re <q> [-Bp | -Bs | -Ap] [-l] -a <alpha>\n\n");
    fprintf(stderr, "  -Be, Bu             = Bessel, Butterworth\n");
    fprintf(stderr, "  -Ch <r>             = Chebyshev (r = dB ripple)\n");
    fprintf(stderr, "  -Pi                 = Proportional-Integral\n");
    fprintf(stderr, "  -Re <q>             = 2-pole resonator (q = Q-factor)\n");
    fprintf(stderr, "  -Lp, Hp, Bp, Bs, Ap = lowpass, highpass, bandpass, bandstop, allpass\n");
    fprintf(stderr, "  -p                  = use listed poles only (ni = 0 .. order-1)\n");
    fprintf(stderr, "  -l                  = just list <order> parameters\n");
    fprintf(stderr, "  -w                  = don't pre-warp frequencies\n");
    fprintf(stderr, "  -z                  = use matched z-transform\n");
    fprintf(stderr, "  -Z                  = additional z-plane zero\n");
    fprintf(stderr, "  order = 1..%d;  alpha = f(corner)/f(sample)\n\n", MAXORDER);
    throw std::logic_error("mkfilter::MKFilter: Invalid usage.");
  }

static void checkoptions(struct Data_t& data)
  { data.optsok = true;
    unless (onebit(data.options & (opt_be | opt_bu | opt_ch | opt_re | opt_pi)))
      opterror(data, "must specify exactly one of -Be, -Bu, -Ch, -Re, -Pi");
    if (data.options & opt_re)
      { unless (onebit(data.options & (opt_bp | opt_bs | opt_ap)))
	  opterror(data, "must specify exactly one of -Bp, -Bs, -Ap with -Re");
	if (data.options & (opt_lp | opt_hp | opt_o | opt_p | opt_w | opt_z))
	  opterror(data, "can't use -Lp, -Hp, -o, -p, -w, -z with -Re");
      }
    else if (data.options & opt_pi)
      { if (data.options & (opt_lp | opt_hp | opt_bp | opt_bs | opt_ap))
	  opterror(data, "-Lp, -Hp, -Bp, -Bs, -Ap illegal in conjunction with -Pi");
	unless ((data.options & opt_o) && (data.order == 1)) opterror(data, "-Pi implies -o 1");
      }
    else
      { unless (onebit(data.options & (opt_lp | opt_hp | opt_bp | opt_bs)))
	  opterror(data, "must specify exactly one of -Lp, -Hp, -Bp, -Bs");
	if (data.options & opt_ap) opterror(data, "-Ap implies -Re");
	if (data.options & opt_o)
	  { unless (data.order >= 1 && data.order <= MAXORDER) opterror(data, "order must be in range 1 .. %d", MAXORDER);
	    if (data.options & opt_p)
	      { uint m = (1 << data.order) - 1; /* "order" bits set */
		if ((data.polemask & ~m) != 0)
		  opterror(data, "order=%d, so args to -p must be in range 0 .. %d", data.order, data.order-1);
	      }
	  }
	else opterror(data, "must specify -o");
      }
    unless (data.options & opt_a) opterror(data, "must specify -a");
    unless (data.optsok) throw std::logic_error("mkfilter::MKFilter: Invalid usage.");
  }

static void opterror(struct Data_t& data, const char *msg, int p1, int p2)
  { fprintf(stderr, "mkfilter: "); fprintf(stderr, msg, p1, p2); putc('\n', stderr);
    data.optsok = false;
  }

static void setdefaults(struct Data_t& data)
  { unless (data.options & opt_p) data.polemask = ~0; /* use all poles */
    unless (data.options & (opt_bp | opt_bs)) data.raw_alpha2 = data.raw_alpha1;
  }

static void compute_s(struct Data_t& data) /* compute S-plane poles for prototype LP filter */
  { data.splane.numpoles = 0;
    if (data.options & opt_be)
      { /* Bessel filter */
	int p = (data.order*data.order)/4; /* ptr into table */
	if (data.order & 1) choosepole(data, bessel_poles[p++]);
	for (int i = 0; i < data.order/2; i++)
	  { choosepole(data, bessel_poles[p]);
	    choosepole(data, cconj(bessel_poles[p]));
	    p++;
	  }
      }
    if (data.options & (opt_bu | opt_ch))
      { /* Butterworth filter */
	for (int i = 0; i < 2*data.order; i++)
	  { double theta = (data.order & 1) ? (i*PI) / data.order : ((i+0.5)*PI) / data.order;
	    choosepole(data, expj(theta));
	  }
      }
    if (data.options & opt_ch)
      { /* modify for Chebyshev (p. 136 DeFatta et al.) */
	if (data.chebrip >= 0.0)
	  { fprintf(stderr, "mkfilter: Chebyshev ripple is %g dB; must be .lt. 0.0\n", data.chebrip);
            throw std::logic_error("mkfilter::MKFilter: Invalid usage.");
	  }
	double rip = pow(10.0, -data.chebrip / 10.0);
	double eps = sqrt(rip - 1.0);
	double y = asinh(1.0 / eps) / (double) data.order;
	if (y <= 0.0)
	  { fprintf(stderr, "mkfilter: bug: Chebyshev y=%g; must be .gt. 0.0\n", y);
            throw std::logic_error("mkfilter::MKFilter: Invalid usage.");
	  }
	for (int i = 0; i < data.splane.numpoles; i++)
	  { data.splane.poles[i].re *= sinh(y);
	    data.splane.poles[i].im *= cosh(y);
	  }
      }
  }

static void choosepole(struct Data_t& data, complex z)
  { if (z.re < 0.0)
      { if (data.polemask & 1) data.splane.poles[data.splane.numpoles++] = z;
	data.polemask >>= 1;
      }
  }

static void prewarp(struct Data_t& data)
  { /* for bilinear transform, perform pre-warp on alpha values */
    if (data.options & (opt_w | opt_z))
      { data.warped_alpha1 = data.raw_alpha1;
	data.warped_alpha2 = data.raw_alpha2;
      }
    else
      { data.warped_alpha1 = tan(PI * data.raw_alpha1) / PI;
	data.warped_alpha2 = tan(PI * data.raw_alpha2) / PI;
      }
  }

static void normalize(struct Data_t& data)		/* called for trad, not for -Re or -Pi */
  { double w1 = TWOPI * data.warped_alpha1;
    double w2 = TWOPI * data.warped_alpha2;
    /* transform prototype into appropriate filter type (lp/hp/bp/bs) */
    switch (data.options & (opt_lp | opt_hp | opt_bp| opt_bs))
      { case opt_lp:
	  { for (int i = 0; i < data.splane.numpoles; i++) data.splane.poles[i] = data.splane.poles[i] * w1;
	    data.splane.numzeros = 0;
	    break;
	  }

	case opt_hp:
	  { int i;
	    for (i=0; i < data.splane.numpoles; i++) data.splane.poles[i] = w1 / data.splane.poles[i];
	    for (i=0; i < data.splane.numpoles; i++) data.splane.zeros[i] = 0.0;	 /* also N zeros at (0,0) */
	    data.splane.numzeros = data.splane.numpoles;
	    break;
	  }

	case opt_bp:
	  { double w0 = sqrt(w1*w2), bw = w2-w1; int i;
	    for (i=0; i < data.splane.numpoles; i++)
	      { complex hba = 0.5 * (data.splane.poles[i] * bw);
		complex temp = csqrt(1.0 - sqr(w0 / hba));
		data.splane.poles[i] = hba * (1.0 + temp);
		data.splane.poles[data.splane.numpoles+i] = hba * (1.0 - temp);
	      }
	    for (i=0; i < data.splane.numpoles; i++) data.splane.zeros[i] = 0.0;	 /* also N zeros at (0,0) */
	    data.splane.numzeros = data.splane.numpoles;
	    data.splane.numpoles *= 2;
	    break;
	  }

	case opt_bs:
	  { double w0 = sqrt(w1*w2), bw = w2-w1; int i;
	    for (i=0; i < data.splane.numpoles; i++)
	      { complex hba = 0.5 * (bw / data.splane.poles[i]);
		complex temp = csqrt(1.0 - sqr(w0 / hba));
		data.splane.poles[i] = hba * (1.0 + temp);
		data.splane.poles[data.splane.numpoles+i] = hba * (1.0 - temp);
	      }
	    for (i=0; i < data.splane.numpoles; i++)	   /* also 2N zeros at (0, +-w0) */
	      { data.splane.zeros[i] = complex(0.0, +w0);
		data.splane.zeros[data.splane.numpoles+i] = complex(0.0, -w0);
	      }
	    data.splane.numpoles *= 2;
	    data.splane.numzeros = data.splane.numpoles;
	    break;
	  }
      }
  }

static void compute_z_blt(struct Data_t& data) /* given S-plane poles & zeros, compute Z-plane poles & zeros, by bilinear transform */
  { int i;
    data.zplane.numpoles = data.splane.numpoles;
    data.zplane.numzeros = data.splane.numzeros;
    for (i=0; i < data.zplane.numpoles; i++) data.zplane.poles[i] = blt(data.splane.poles[i]);
    for (i=0; i < data.zplane.numzeros; i++) data.zplane.zeros[i] = blt(data.splane.zeros[i]);
    while (data.zplane.numzeros < data.zplane.numpoles) data.zplane.zeros[data.zplane.numzeros++] = -1.0;
  }

static complex blt(complex pz)
  { return (2.0 + pz) / (2.0 - pz);
  }

static void compute_z_mzt(struct Data_t& data) /* given S-plane poles & zeros, compute Z-plane poles & zeros, by matched z-transform */
  { int i;
    data.zplane.numpoles = data.splane.numpoles;
    data.zplane.numzeros = data.splane.numzeros;
    for (i=0; i < data.zplane.numpoles; i++) data.zplane.poles[i] = cexp(data.splane.poles[i]);
    for (i=0; i < data.zplane.numzeros; i++) data.zplane.zeros[i] = cexp(data.splane.zeros[i]);
  }

static void compute_notch(struct Data_t& data)
  { /* compute Z-plane pole & zero positions for bandstop resonator (notch filter) */
    compute_bpres(data);		/* iterate to place poles */
    double theta = TWOPI * data.raw_alpha1;
    complex zz = expj(theta);	/* place zeros exactly */
    data.zplane.zeros[0] = zz; data.zplane.zeros[1] = cconj(zz);
  }

static void compute_apres(struct Data_t& data)
  { /* compute Z-plane pole & zero positions for allpass resonator */
    compute_bpres(data);		/* iterate to place poles */
    data.zplane.zeros[0] = reflect(data.zplane.poles[0]);
    data.zplane.zeros[1] = reflect(data.zplane.poles[1]);
  }

static complex reflect(complex z)
  { double r = hypot(z);
    return z / sqr(r);
  }

static void compute_bpres(struct Data_t& data)
  { /* compute Z-plane pole & zero positions for bandpass resonator */
    data.zplane.numpoles = data.zplane.numzeros = 2;
    data.zplane.zeros[0] = 1.0; data.zplane.zeros[1] = -1.0;
    double theta = TWOPI * data.raw_alpha1; /* where we want the peak to be */
    if (data.infq)
      { /* oscillator */
	complex zp = expj(theta);
	data.zplane.poles[0] = zp; data.zplane.poles[1] = cconj(zp);
      }
    else
      { /* must iterate to find exact pole positions */
	complex topcoeffs[MAXPZ+1]; expand(data.zplane.zeros, data.zplane.numzeros, topcoeffs);
	double r = exp(-theta / (2.0 * data.qfactor));
	double thm = theta, th1 = 0.0, th2 = PI;
	bool cvg = false;
	for (int i=0; i < 50 && !cvg; i++)
	  { complex zp = r * expj(thm);
	    data.zplane.poles[0] = zp; data.zplane.poles[1] = cconj(zp);
	    complex botcoeffs[MAXPZ+1]; expand(data.zplane.poles, data.zplane.numpoles, botcoeffs);
	    complex g = evaluate(topcoeffs, data.zplane.numzeros, botcoeffs, data.zplane.numpoles, expj(theta));
	    double phi = g.im / g.re; /* approx to atan2 */
	    if (phi > 0.0) th2 = thm; else th1 = thm;
	    if (fabs(phi) < EPS) cvg = true;
	    thm = 0.5 * (th1+th2);
	  }
	unless (cvg) fprintf(stderr, "mkfilter: warning: failed to converge\n");
      }
  }

static void add_extra_zero(struct Data_t& data)
  { if (data.zplane.numzeros+2 > MAXPZ)
      { fprintf(stderr, "mkfilter: too many zeros; can't do -Z\n");
        throw std::logic_error("mkfilter::MKFilter: Invalid usage.");
      }
    double theta = TWOPI * data.raw_alphaz;
    complex zz = expj(theta);
    data.zplane.zeros[data.zplane.numzeros++] = zz;
    data.zplane.zeros[data.zplane.numzeros++] = cconj(zz);
    while (data.zplane.numpoles < data.zplane.numzeros) data.zplane.poles[data.zplane.numpoles++] = 0.0;	 /* ensure causality */
  }

static void expandpoly(struct Data_t& data) /* given Z-plane poles & zeros, compute top & bot polynomials in Z, and then recurrence relation */
  { complex topcoeffs[MAXPZ+1], botcoeffs[MAXPZ+1]; int i;
    expand(data.zplane.zeros, data.zplane.numzeros, topcoeffs);
    expand(data.zplane.poles, data.zplane.numpoles, botcoeffs);
    data.dc_gain = evaluate(topcoeffs, data.zplane.numzeros, botcoeffs, data.zplane.numpoles, 1.0);
    double theta = TWOPI * 0.5 * (data.raw_alpha1 + data.raw_alpha2); /* "jwT" for centre freq. */
    data.fc_gain = evaluate(topcoeffs, data.zplane.numzeros, botcoeffs, data.zplane.numpoles, expj(theta));
    data.hf_gain = evaluate(topcoeffs, data.zplane.numzeros, botcoeffs, data.zplane.numpoles, -1.0);
    for (i = 0; i <= data.zplane.numzeros; i++) data.xcoeffs[i] = +(topcoeffs[i].re / botcoeffs[data.zplane.numpoles].re);
    for (i = 0; i <= data.zplane.numpoles; i++) data.ycoeffs[i] = -(botcoeffs[i].re / botcoeffs[data.zplane.numpoles].re);
  }

static void expand(complex pz[], int npz, complex coeffs[])
  { /* compute product of poles or zeros as a polynomial of z */
    int i;
    coeffs[0] = 1.0;
    for (i=0; i < npz; i++) coeffs[i+1] = 0.0;
    for (i=0; i < npz; i++) multin(pz[i], npz, coeffs);
    /* check computed coeffs of z^k are all real */
    for (i=0; i < npz+1; i++)
      { if (fabs(coeffs[i].im) > EPS)
	  { fprintf(stderr, "mkfilter: coeff of z^%d is not real; poles/zeros are not complex conjugates\n", i);
            throw std::logic_error("mkfilter::MKFilter: Numeric error.");
	  }
      }
  }

static void multin(complex w, int npz, complex coeffs[])
  { /* multiply factor (z-w) into coeffs */
    complex nw = -w;
    for (int i = npz; i >= 1; i--) coeffs[i] = (nw * coeffs[i]) + coeffs[i-1];
    coeffs[0] = nw * coeffs[0];
  }

