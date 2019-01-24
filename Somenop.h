#ifndef SOMENOPH
#define SOMENOPH

#include <intrins.h>	/* _nop_ */

#define Somenop();       _nop_();_nop_();_nop_();_nop_();_nop_();
#define Somenop10();	 Somenop();Somenop();
#define Somenop25();	 Somenop();Somenop();Somenop();Somenop();Somenop();
#define Somenop50();	 Somenop25();Somenop25();

#endif