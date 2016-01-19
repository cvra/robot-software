#include "CppUTest/TestHarness.h"
#include "../parameter.h"
#include "../parameter_print.h"

static int outbuffer_pos;
static char outbuffer[1000];

TEST_GROUP(Print)
{


    parameter_namespace_t rootns;
    parameter_t param_scalar;
    parameter_t param_integer;
    parameter_t param_boolean;
    parameter_namespace_t sub1;
    parameter_t param_string;
    parameter_t param_not_set;
    parameter_namespace_t sub1sub1;
    parameter_t param_vect;
    parameter_namespace_t sub2;
    parameter_t param_var_vect;

    char strbuf[10];
    float vectbuf[3];
    float varvectbuf[10];
    void setup(void)
    {
        outbuffer_pos = 0;
        vectbuf[0] = 1;
        vectbuf[1] = 2;
        vectbuf[2] = 3;
        varvectbuf[0] = 33;
        varvectbuf[1] = 44;
        parameter_namespace_declare(&rootns, NULL, NULL);
        parameter_scalar_declare_with_default(&param_scalar, &rootns, "param_scalar", 3.1415926536);
        parameter_integer_declare_with_default(&param_integer, &rootns, "param_integer", 42);
        parameter_boolean_declare_with_default(&param_boolean, &rootns, "param_boolean", true);
        parameter_namespace_declare(&sub1, &rootns, "sub1");
        parameter_string_declare_with_default(&param_string, &sub1, "param_string", strbuf, sizeof(strbuf), "abcd");
        parameter_scalar_declare(&param_not_set, &sub1, "param_not_set");
        parameter_namespace_declare(&sub1sub1, &sub1, "sub1sub1");
        parameter_vector_declare_with_default(&param_vect, &sub1sub1, "param_vect", vectbuf, 3);
        parameter_namespace_declare(&sub2, &rootns, "sub2");
        parameter_variable_vector_declare_with_default(&param_var_vect, &sub2, "param_var_vect", varvectbuf, sizeof(varvectbuf)/sizeof(float), 2);
    }
};

int printfn(void *arg, const char *fmt, ...)
{
    (void) arg;

    va_list args;
    va_start(args, fmt);
    int remaining = sizeof(outbuffer) - outbuffer_pos;
    if (remaining < 0) {
        return 0;
    }
    outbuffer_pos += vsnprintf(&outbuffer[outbuffer_pos], remaining,fmt, args);
    va_end(args);
    return 0;
}


const char *expected =
"param_boolean: true\n"
"param_integer: 42\n"
"param_scalar: 3.141593\n"
"sub2:\n"
"  param_var_vect: [33.000000, 44.000000]\n"
"sub1:\n"
"  param_not_set: ~\n"
"  param_string: \"abcd\"\n"
"  sub1sub1:\n"
"    param_vect: [1.000000, 2.000000, 3.000000]\n";

TEST(Print, tst1)
{
    parameter_print(&rootns, (parameter_printfn_t)printfn, (void*)NULL);
    outbuffer[outbuffer_pos] = '\0';
    // printf("%s", outbuffer);
    STRCMP_EQUAL(expected, outbuffer);
}
