#include "CppUTest/TestHarness.h"
#include "parameter.h"

TEST_GROUP(ParameterNamespaceInit)
{
    parameter_namespace_t rootns;
    void setup(void)
    {
        parameter_namespace_declare(&rootns, NULL, NULL);
    }
};

TEST(ParameterNamespaceInit, NamespaceCreateRoot)
{
    parameter_namespace_t ns;
    parameter_namespace_declare(&ns, NULL, NULL);
    CHECK_EQUAL(NULL, ns.id);
    CHECK_EQUAL(NULL, ns.parent);
    CHECK_EQUAL(NULL, ns.subspaces);
    CHECK_EQUAL(NULL, ns.next);
    CHECK_EQUAL(NULL, ns.parameter_list);
    CHECK_EQUAL(0, ns.changed_cnt);
}

TEST(ParameterNamespaceInit, NamespaceCreate)
{
    parameter_namespace_t ns;
    parameter_namespace_declare(&ns, &rootns, "test");
    STRCMP_EQUAL("test", ns.id);
    CHECK_EQUAL(&rootns, ns.parent);
    CHECK_EQUAL(NULL, ns.subspaces);
    CHECK_EQUAL(NULL, ns.parameter_list);
    CHECK_EQUAL(0, ns.changed_cnt);
    // check if correctly linked in parent
    CHECK_EQUAL(NULL, ns.next);
    CHECK_EQUAL(&ns, rootns.subspaces);
}


TEST(ParameterNamespaceInit, NamespaceCreateMultiple)
{
    parameter_namespace_t ns1;
    parameter_namespace_t ns2;
    parameter_namespace_declare(&ns1, &rootns, "test1");
    parameter_namespace_declare(&ns2, &rootns, "test2");
    CHECK_EQUAL(&rootns, ns1.parent);
    CHECK_EQUAL(&rootns, ns2.parent);
    // check if correctly linked in parent
    CHECK_EQUAL(NULL, ns1.next);
    CHECK_EQUAL(&ns1, ns2.next);
    CHECK_EQUAL(&ns2, rootns.subspaces);
}


TEST_GROUP(ParameterInit)
{
    parameter_namespace_t ns;
    void setup(void)
    {
        parameter_namespace_declare(&ns, NULL, NULL);
    }
};

TEST(ParameterInit, ParameterCreate)
{
    parameter_t p;
    _parameter_declare(&p, &ns, "test");
    STRCMP_EQUAL("test", p.id);
    CHECK_EQUAL(&ns, p.ns);
    CHECK_EQUAL(false, p.changed);
    CHECK_EQUAL(false, p.set);
    // check if correctly linked in parent
    CHECK_EQUAL(NULL, p.next);
    CHECK_EQUAL(&p, ns.parameter_list);
}

TEST(ParameterInit, ParameterCreateMulitple)
{
    parameter_t p1;
    parameter_t p2;
    _parameter_declare(&p1, &ns, "test1");
    _parameter_declare(&p2, &ns, "test2");
    CHECK_EQUAL(&ns, p1.ns);
    CHECK_EQUAL(&ns, p2.ns);
    // check if correctly linked in parent
    CHECK_EQUAL(NULL, p1.next);
    CHECK_EQUAL(&p1, p2.next);
    CHECK_EQUAL(&p2, ns.parameter_list);
}


TEST_GROUP(ParameterTree)
{
    parameter_namespace_t rootns;
    parameter_namespace_t a;
    parameter_namespace_t a1;
    parameter_namespace_t a2;
    parameter_namespace_t b;
    parameter_namespace_t b1;
    parameter_namespace_t b1i;
    parameter_namespace_t b1ii;
    parameter_t p_a2_x;
    parameter_t p_a2_y;
    parameter_t p_a2_z;
    parameter_t p_root_x;
    parameter_t p_b1i_x;
    void setup(void)
    {
        parameter_namespace_declare(&rootns, NULL, NULL);
        parameter_namespace_declare(&a, &rootns, "test_a");
        parameter_namespace_declare(&a1, &a, "eins");
        parameter_namespace_declare(&a2, &a, "zwei");
        parameter_namespace_declare(&b, &rootns, "test_b");
        parameter_namespace_declare(&b1, &b, "eins");
        parameter_namespace_declare(&b1i, &b1, "I");
        parameter_namespace_declare(&b1ii, &b1, "II");
        _parameter_declare(&p_a2_x, &a2, "x");
        _parameter_declare(&p_a2_y, &a2, "y");
        _parameter_declare(&p_a2_z, &a2, "z");
        _parameter_declare(&p_root_x, &rootns, "x");
        _parameter_declare(&p_b1i_x, &b1i, "x");
    }
};

TEST(ParameterTree, NamespaceFind)
{
    CHECK_EQUAL(NULL, parameter_namespace_find(&rootns, "does/not/exist"));
    CHECK_EQUAL(NULL, parameter_namespace_find(&rootns, "test"));
    CHECK_EQUAL(&a, parameter_namespace_find(&rootns, "test_a"));
    CHECK_EQUAL(&a, parameter_namespace_find(&rootns, "test_a/"));
    CHECK_EQUAL(&a, parameter_namespace_find(&rootns, "/test_a/"));
    CHECK_EQUAL(&a1, parameter_namespace_find(&rootns, "/test_a//eins/"));
    CHECK_EQUAL(&a2, parameter_namespace_find(&rootns, "test_a/zwei"));
    CHECK_EQUAL(&b1i, parameter_namespace_find(&rootns, "test_b/eins/I"));
    CHECK_EQUAL(&b1ii, parameter_namespace_find(&rootns, "test_b/eins/II"));
    CHECK_EQUAL(&b1ii, parameter_namespace_find(&b, "eins/II"));
}

TEST(ParameterTree, ParameterFind)
{
    CHECK_EQUAL(NULL, parameter_find(&rootns, "test"));
    CHECK_EQUAL(NULL, parameter_find(&b1ii, "test"));
    CHECK_EQUAL(NULL, parameter_find(&b1ii, "x"));
    CHECK_EQUAL(&p_a2_x, parameter_find(&rootns, "test_a/zwei/x"));
    CHECK_EQUAL(&p_a2_y, parameter_find(&rootns, "test_a/zwei/y"));
    CHECK_EQUAL(&p_a2_z, parameter_find(&rootns, "test_a/zwei/z"));
    CHECK_EQUAL(&p_root_x, parameter_find(&rootns, "/x"));
    CHECK_EQUAL(&p_b1i_x, parameter_find(&rootns, "test_b/eins/I/x"));
}


#include "CppUTest/CommandLineTestRunner.h"

int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}
