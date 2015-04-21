#include "CppUTest/TestHarness.h"
#include "../parameter.h"

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
    POINTERS_EQUAL(NULL, ns.id);
    POINTERS_EQUAL(NULL, ns.parent);
    POINTERS_EQUAL(NULL, ns.subspaces);
    POINTERS_EQUAL(NULL, ns.next);
    POINTERS_EQUAL(NULL, ns.parameter_list);
    CHECK_EQUAL(0, ns.changed_cnt);
}

TEST(ParameterNamespaceInit, NamespaceCreate)
{
    parameter_namespace_t ns;
    parameter_namespace_declare(&ns, &rootns, "test");
    STRCMP_EQUAL("test", ns.id);
    POINTERS_EQUAL(&rootns, ns.parent);
    POINTERS_EQUAL(NULL, ns.subspaces);
    POINTERS_EQUAL(NULL, ns.parameter_list);
    CHECK_EQUAL(0, ns.changed_cnt);
    // check if correctly linked in parent
    POINTERS_EQUAL(NULL, ns.next);
    POINTERS_EQUAL(&ns, rootns.subspaces);
}


TEST(ParameterNamespaceInit, NamespaceCreateMultiple)
{
    parameter_namespace_t ns1;
    parameter_namespace_t ns2;
    parameter_namespace_declare(&ns1, &rootns, "test1");
    parameter_namespace_declare(&ns2, &rootns, "test2");
    POINTERS_EQUAL(&rootns, ns1.parent);
    POINTERS_EQUAL(&rootns, ns2.parent);
    // check if correctly linked in parent
    POINTERS_EQUAL(NULL, ns1.next);
    POINTERS_EQUAL(&ns1, ns2.next);
    POINTERS_EQUAL(&ns2, rootns.subspaces);
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
    POINTERS_EQUAL(&ns, p.ns);
    CHECK_FALSE(p.changed);
    CHECK_FALSE(p.defined);
    // check if correctly linked in parent
    POINTERS_EQUAL(NULL, p.next);
    POINTERS_EQUAL(&p, ns.parameter_list);
}

TEST(ParameterInit, ParameterCreateMulitple)
{
    parameter_t p1;
    parameter_t p2;
    _parameter_declare(&p1, &ns, "test1");
    _parameter_declare(&p2, &ns, "test2");
    POINTERS_EQUAL(&ns, p1.ns);
    POINTERS_EQUAL(&ns, p2.ns);
    // check if correctly linked in parent
    POINTERS_EQUAL(NULL, p1.next);
    POINTERS_EQUAL(&p1, p2.next);
    POINTERS_EQUAL(&p2, ns.parameter_list);
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
    POINTERS_EQUAL(NULL, parameter_namespace_find(&rootns, "does/not/exist"));
    POINTERS_EQUAL(NULL, parameter_namespace_find(&rootns, "test"));
    POINTERS_EQUAL(&a, parameter_namespace_find(&rootns, "test_a"));
    POINTERS_EQUAL(&a, parameter_namespace_find(&rootns, "test_a/"));
    POINTERS_EQUAL(&a, parameter_namespace_find(&rootns, "/test_a/"));
    POINTERS_EQUAL(&a1, parameter_namespace_find(&rootns, "/test_a//eins/"));
    POINTERS_EQUAL(&a2, parameter_namespace_find(&rootns, "test_a/zwei"));
    POINTERS_EQUAL(&b1i, parameter_namespace_find(&rootns, "test_b/eins/I"));
    POINTERS_EQUAL(&b1ii, parameter_namespace_find(&rootns, "test_b/eins/II"));
    POINTERS_EQUAL(&b1ii, parameter_namespace_find(&b, "eins/II"));
}

TEST(ParameterTree, ParameterFind)
{
    POINTERS_EQUAL(NULL, parameter_find(&rootns, "test"));
    POINTERS_EQUAL(NULL, parameter_find(&b1ii, "test"));
    POINTERS_EQUAL(NULL, parameter_find(&b1ii, "x"));
    POINTERS_EQUAL(&p_a2_x, parameter_find(&rootns, "test_a/zwei/x"));
    POINTERS_EQUAL(&p_a2_y, parameter_find(&rootns, "test_a/zwei/y"));
    POINTERS_EQUAL(&p_a2_z, parameter_find(&rootns, "test_a/zwei/z"));
    POINTERS_EQUAL(&p_root_x, parameter_find(&rootns, "/x"));
    POINTERS_EQUAL(&p_b1i_x, parameter_find(&rootns, "test_b/eins/I/x"));
}

TEST(ParameterTree, ParameterSetClear)
{
    CHECK_FALSE(parameter_changed(&p_root_x));
    CHECK_FALSE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_set(&p_root_x);
    CHECK_TRUE(parameter_changed(&p_root_x));
    CHECK_TRUE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_clear(&p_root_x);
    CHECK_FALSE(parameter_changed(&p_root_x));
    CHECK_FALSE(parameter_namespace_contains_changed(&rootns));
}

TEST(ParameterTree, ParameterSetClearHierarchical)
{
    CHECK_FALSE(parameter_changed(&p_a2_x));
    CHECK_FALSE(parameter_namespace_contains_changed(&a2));
    CHECK_FALSE(parameter_namespace_contains_changed(&a));
    CHECK_FALSE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_set(&p_a2_x);
    CHECK_FALSE(parameter_changed(&p_a2_y));
    CHECK_TRUE(parameter_changed(&p_a2_x));
    CHECK_TRUE(parameter_namespace_contains_changed(&a2));
    CHECK_TRUE(parameter_namespace_contains_changed(&a));
    CHECK_TRUE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_set(&p_a2_y);
    CHECK_TRUE(parameter_changed(&p_a2_x));
    CHECK_TRUE(parameter_changed(&p_a2_y));
    CHECK_TRUE(parameter_namespace_contains_changed(&a2));
    CHECK_TRUE(parameter_namespace_contains_changed(&a));
    CHECK_TRUE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_clear(&p_a2_x);
    CHECK_FALSE(parameter_changed(&p_a2_x));
    CHECK_TRUE(parameter_changed(&p_a2_y));
    CHECK_TRUE(parameter_namespace_contains_changed(&a2));
    CHECK_TRUE(parameter_namespace_contains_changed(&a));
    CHECK_TRUE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_clear(&p_a2_y);
    _parameter_changed_set(&p_root_x);
    CHECK_FALSE(parameter_changed(&p_a2_x));
    CHECK_FALSE(parameter_changed(&p_a2_y));
    CHECK_FALSE(parameter_namespace_contains_changed(&a2));
    CHECK_FALSE(parameter_namespace_contains_changed(&a));
    CHECK_TRUE(parameter_namespace_contains_changed(&rootns));
    _parameter_changed_clear(&p_root_x);
    CHECK_FALSE(parameter_namespace_contains_changed(&rootns));
    CHECK_FALSE(parameter_namespace_contains_changed(&a));
}

TEST(ParameterTree, ParameterDefined)
{
    CHECK_FALSE(parameter_defined(&p_a2_z));
    _parameter_changed_set(&p_a2_z);
    CHECK_TRUE(parameter_defined(&p_a2_z));
    _parameter_changed_clear(&p_a2_z);
    CHECK_TRUE(parameter_defined(&p_a2_z));
}
