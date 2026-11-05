#include "include/params.hpp"

#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
  using namespace params;

  Property<std::string> test;
  // just assign property with the value of the same type
  test = (std::string) "This is a test";
  // convert back to the original type
  std::string strtest = test;

  std::cout << test << std::endl;
  std::cout << strtest << std::endl;

  // the same works for enums and other primitive types
  enum ExampleEnum { E0, E1, E2 };
  Property<ExampleEnum> enumTest;
  enumTest = E1;
  ExampleEnum realEnum = enumTest;
  std::cout << realEnum << std::endl;

  // create a group of parameters
  struct ParentSettings : public Group {
    // make sure to include the constructor
    using Group::Group;
    // you can create subgroups within groups
    struct MySettings1 : public Group {
      using Group::Group;
      Property<std::string> test{"Value", "test", this};
      Property<ExampleEnum> enumProperty{E1, "enum_property", this};
    } mySettings1{"mySettings1", this};

    struct MySettings2 : public Group {
      using Group::Group;
      Property<unsigned int> uint{123u, "uint", this};
      Property<double> pi{M_PI, "pi", this};
      Property<std::vector<double>> numbers{{0, 1, 2, 3}, "numbers", this};
    } mySettings2{"mySettings2", this};
  } mainSettings{"mainSettings"};
  std::cout << mainSettings << std::endl;

  // retrieve a value from a property within a group
  assert(mainSettings.mySettings1.enumProperty == E1);

  // serialize settings to JSON
  nlohmann::json j = mainSettings;
  std::cout << j.dump() << std::endl;

  mainSettings.mySettings1.test = "This change will be overwritten.";

  mainSettings.load(j);
  std::cout << mainSettings;

  return EXIT_SUCCESS;
}
