#include "CppUTest/TestHarness.h"
#include "parameter.h"

TEST_GROUP(ParamNamespaceInit)
{
    parameter_namespace_t rootns;
    void setup(void)
    {
        parameter_namespace_declare(&rootns, NULL, NULL);
    }
};

TEST(ParamNamespaceInit, NamespaceCreateRoot)
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

TEST(ParamNamespaceInit, NamespaceCreateNoParam)
{
    parameter_namespace_t ns;
    parameter_namespace_declare(&ns, &rootns, "test");
    STRCMP_EQUAL("test", ns.id);
    CHECK_EQUAL(&rootns, ns.parent);
    CHECK_EQUAL(NULL, ns.subspaces);
    CHECK_EQUAL(NULL, ns.next);
    CHECK_EQUAL(NULL, ns.parameter_list);
    CHECK_EQUAL(0, ns.changed_cnt);
    // check if correctly linked in parent
    CHECK_EQUAL(&ns, rootns.subspaces);
}

TEST(ParamNamespaceInit, NamespaceCreateParam)
{
    parameter_namespace_t ns;
    parameter_namespace_declare(&ns, &rootns, "test");
    STRCMP_EQUAL("test", ns.id);
    CHECK_EQUAL(&rootns, ns.parent);
    CHECK_EQUAL(NULL, ns.subspaces);
    CHECK_EQUAL(NULL, ns.next);
    CHECK_EQUAL(NULL, ns.parameter_list);
    CHECK_EQUAL(0, ns.changed_cnt);
    // check if correctly linked in parent
    CHECK_EQUAL(&ns, rootns.subspaces);
}

TEST(ParamNamespaceInit, NamespaceCreateMultiple)
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


TEST_GROUP(ParamNamespace)
{
    parameter_namespace_t rootns;
    parameter_namespace_t a;
    parameter_namespace_t a1;
    parameter_namespace_t a2;
    parameter_namespace_t b;
    parameter_namespace_t b1;
    parameter_namespace_t b1i;
    parameter_namespace_t b1ii;
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
    }
};

TEST(ParamNamespace, NamespaceGet)
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



#include "CppUTest/CommandLineTestRunner.h"

int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}
