
#include <iostream>

#include "test_plugins/mainFoo.h"

extern "C" int pluginFoo() 
{
  std::cerr << "Plugin calling main foo\n";
  int a = mainFoo();
  std::cerr << "Call from main program returned " << a << "\n";

  std::cerr << "Fctn foo from plugin \n";
  return 5;
}
