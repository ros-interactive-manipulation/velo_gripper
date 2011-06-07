
#include <dlfcn.h>
#include <iostream>

#include "test_plugins/mainFoo.h"

typedef int (*IntFctn)();

int main(int argc, char **argv)
{
  void* handle = dlopen("libplugin.so", RTLD_LAZY | RTLD_GLOBAL);
  char *errstr = dlerror();
  if (!handle) {
    std::cerr << "Failed to open dynamic library\n";
    if (errstr) std::cerr << "Error: " << errstr <<"\n";
    return -1;
  }

  IntFctn intFctn;
  *(void **)(&intFctn) = dlsym(handle,"pluginFoo");
  if (dlerror()) {
    std::cerr << "Could not load symbol pluginFoo\n";
    return -1;
  }

  int a = mainFoo();

  a = (*intFctn)();
  std::cerr << "pluginFoo returned " << a << "\n";
  dlclose(handle);
  return 0;
}
