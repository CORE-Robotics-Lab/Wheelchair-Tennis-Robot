#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#ifdef DEBUG_PRINT
#define debug_setup() \
    SerialUSB1.begin(115200);
#define debug_print(x) \
    SerialUSB1.print(x);
#define debug_println(x) \
    SerialUSB1.println(x);
#define debug_printm()          \
    SerialUSB1.print(micros()); \
    SerialUSB1.print(" ");
#define debug_printlnm(x)       \
    SerialUSB1.print(micros()); \
    SerialUSB1.print(" ");      \
    SerialUSB1.println(x);
#define debug_printt(x)  \
    SerialUSB1.print(x); \
    SerialUSB1.print('\t');
#define debug_printf(x, y) \
    SerialUSB1.print(x, y);
#define debug_printlnf(x, y)    \
    SerialUSB1.print(micros()); \
    SerialUSB1.print(" ");      \
    SerialUSB1.println(x, y);
#define debug_printv(x, y)  \
    SerialUSB1.print(x);    \
    SerialUSB1.print(": "); \
    SerialUSB1.println(y);
#define debug_printlnv(x, y)    \
    SerialUSB1.print(micros()); \
    SerialUSB1.print(" ");      \
    SerialUSB1.print(x);        \
    SerialUSB1.print(": ");     \
    SerialUSB1.println(y);
#else
#define debug_setup()
#define debug_printm()
#define debug_printlnm(x)
#define debug_print(x)
#define debug_println(x)
#define debug_printt(x)
#define debug_printf(x, y)
#define debug_printlnf(x, y)
#define debug_printv(x, y)
#define debug_printlnv(x, y)
#endif

#ifdef LOG_PRINT
#define log_setup() SerialUSB2.begin(115200);
#define log_print(x) SerialUSB2.print(x);
#define log_println(x)          \
    SerialUSB2.print(micros()); \
    SerialUSB2.print(" ");      \
    SerialUSB2.println(x);
#define log_printlnf(x, y)      \
    SerialUSB2.print(micros()); \
    SerialUSB2.print(" ");      \
    SerialUSB2.println(x, y);
#define log_pln() SerialUSB2.println();
#else
#define log_setup()
#define log_print(x)
#define log_println(x)
#endif

#endif
