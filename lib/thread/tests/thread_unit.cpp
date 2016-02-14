#include <ecl/thread/thread.hpp>

#include <CppUTest/TestHarness.h>
#include <CppUTest/CommandLineTestRunner.h>

// Error code helper
// TODO: move it to 'utils' headers and protect with check of
// current test state (enabled or disabled)
//static SimpleString StringFrom(ecl::err err)
//{
//    return SimpleString{ecl::err_to_str(err)};
//}

int test_counter;

int single_thread_fn(void *data)
{
    // Required to check that valid context is passed
    POINTERS_EQUAL(&test_counter, data);
    int *counter = reinterpret_cast< int * >(data);
    (*counter)++;


    return 0;
}

TEST_GROUP(native_thread)
{
    void setup()
    {
        test_counter = 0;
    }

    void teardown()
    {
    }
};

TEST(native_thread, default_thread_is_not_joinable)
{
}

TEST(native_thread, single_thread_is_running)
{
}


int main(int argc, char *argv[])
{
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
