#include <cstdlib>

namespace __gnu_cxx {

void __verbose_terminate_handler()
{
    std::abort();
}

} // namespace __gnu_cxx
